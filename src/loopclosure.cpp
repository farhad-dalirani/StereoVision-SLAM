#include<StereoVisionSLAM/loopclosure.h>
#include<StereoVisionSLAM/visual_odometry.h>
#include<StereoVisionSLAM/slamexception.h>
#include "StereoVisionSLAM/config.h"
#include "StereoVisionSLAM/g2o_types.h"
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>

namespace slam
{

    void LoopClosure::SetCameras(Camera::Ptr left, Camera::Ptr right) 
    {
        cam_left_ = left;
        cam_right_ = right;
    }

    void LoopClosure::SetMap(Map::Ptr map) 
    { 
        map_ = map; 
    }

    void LoopClosure::SetViewer(std::shared_ptr<Viewer> viewer)
    {
        viewer_ = viewer;
    }

    void LoopClosure::SetBackend(std::shared_ptr<Backend> backend)
    {
        backend_ = backend;
    }

    void LoopClosure::SetFrontend(std::shared_ptr<Frontend> frontend)
    {
        frontend_ = frontend;
    }

    LoopClosure::LoopClosure()
    {
        // Set hyperparameters
        keyframes_to_ignore_after_loop_ = Config::Get<int>("keyframes_to_ignore_after_loop");
        potential_loop_weak_threshold_ = Config::Get<float>("potential_loop_weak_threshold");
        potential_loop_strong_threshold_ = Config::Get<float>("potential_loop_strong_threshold");
        max_num_weak_threshold_ = Config::Get<int>("max_num_weak_threshold");
        min_num_acceptable_keypoint_match_ = Config::Get<int>("min_num_acceptable_keypoint_match");
        if(Config::Get<int>("global_pose_graph_optimization") >= 1)
        {
            global_pose_graph_optimization_ = true;
        }
        else
        {
            global_pose_graph_optimization_ = false;
        }
        min_pose_differnece_between_old_new_ = 
            Config::Get<double>("min_pose_differnece_between_old_new");
        max_pose_differnece_between_old_new_ = 
            Config::Get<double>("max_pose_differnece_between_old_new");

        // Keypoint feature descriptor and matcher
        orb_descriptor_ = cv::ORB::create(400);
        matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");

        // Initialize Deep Neural Network for feature extraction from images
        InitialFeatureExtractorNetwork();

        // Initialize backend optimization in a seprate thread
        loopclosure_running_.store(true);
        loopclosure_thread_ = std::thread(std::bind(&LoopClosure::LoopClosurePipeline, this));
    }

    void LoopClosure::InitialFeatureExtractorNetwork()
    {
        /* 
        Create a vision feature extractor using MobileNet V2 pre-trained neural network ONNX file
        with OpenCV DNN. This deep neural network will be utilized to extract feature
        vectors from images, which will help in identifying potential loop closure candidates.
        */

        std::string modelPath = "./dnn_weights/mobilenet_v2.onnx";
        network_ = cv::dnn::readNetFromONNX(modelPath);

        // Check if the model was loaded successfully
        if (network_.empty()) 
        {
            throw SLAMException("Failed to load the Mobilenet V2 ONNX file!");
        }

    }

    void LoopClosure::ExtractFeatureVec(Frame::Ptr frame)
    {
        /* Extract feature vector representation of left image of
         * a frame with the backbone neural network */

        // Make sure image has 3 channel
        cv::Mat dst;
        if(dst.channels() == 1)
        {
            cv::cvtColor(frame->left_img_, dst, cv::COLOR_GRAY2RGB);
        }
        else
        {
            frame->left_img_.copyTo(dst);
        }
        
        /* Blur image for extracting feature vector 
         * with less sensivity to small changes */
        cv::GaussianBlur(dst, dst, cv::Size(7, 7), 0);

        // Preprocess the image: resize, convert to blob, etc.
        cv::Mat blob;
        cv::dnn::blobFromImage(dst, blob, 1.0/255.0, cv::Size(224, 224), 
                                cv::Scalar(0.485, 0.456, 0.406), true, false);

        // Set the input blob to the network
        network_.setInput(blob);

        // Perform the forward pass to get the feature vector
        std::string featureLayerName = "/GlobalAveragePool_output_0"; 
        cv::Mat output = network_.forward(featureLayerName);

        // Copy data from cv::Mat to Eigen vector
        std::memcpy(frame->representation_vec_.data(), output.ptr<float>(), 1280 * sizeof(float));

        // Normalization
	    frame->representation_vec_ /= frame->representation_vec_.norm();
    }

    void LoopClosure::ExtractKeypointsDescriptor(Frame::Ptr frame)
    {
        /* Extract descriptor for keypoint features to later use
         * their discriptors for matching keypoints */

        std::vector<cv::KeyPoint> keypoints;
        
        // Distance to image edge for keypoint feature to have descriptor
        float dis{20};

        // Select non outlier and valid keypoint features
        for(size_t i{0}; i < frame->feature_left_.size(); i++)
        {
            if(frame->feature_left_[i] and (frame->feature_left_[i]->outlier_==false))
            {
                keypoints.push_back(frame->feature_left_[i]->position_); 
            }
        }
        
        // Compute the descriptors
        orb_descriptor_->compute(frame->left_img_, keypoints, frame->descriptor_);

        // Find which keypoint belong to which descriptor
        for(size_t i{0}; i < keypoints.size(); i++)
        {
            for(size_t j{0}; j < frame->feature_left_.size(); j++)
            {
                if((frame->feature_left_[j]->position_.pt.x == keypoints[i].pt.x) and 
                        (frame->feature_left_[j]->position_.pt.y == keypoints[i].pt.y))
                {
                    frame->desc_feat_indx_.push_back(j);
                    break;
                }   
            }
        }        
        
        if(frame->desc_feat_indx_.size() != frame->descriptor_.rows)
        {
            throw SLAMException("Number of calculated descriptors is not equal to number of keypoint features");
        }
    }

    float LoopClosure::SimilarityScore(const Eigen::Matrix<float, 1280, 1> &a, 
                                    const Eigen::Matrix<float, 1280, 1> &b)
    {
        /* Calculate similarity score of two images feature vectors that
         * are extracted with the deep neural network backbone */
        float score = a.transpose() * b;
        return score;
    }

    void LoopClosure::AddNewKeyFrame(Frame::Ptr new_keyframe)
    {
        /* Add a new keyframe to waiting queue to be
         * processed by loop closure pipeline */

        std::unique_lock<std::mutex> lck(list_mutex_);

        /* If recently no loop was detected, add the new 
         * keyframe to the waitlist. */  
        if ((last_closed_keyframe_ == nullptr) || 
             (new_keyframe->keyframe_id_ - last_closed_keyframe_->keyframe_id_ >
                 keyframes_to_ignore_after_loop_))
        {
            waitlist_keyframes_.push_back(new_keyframe);
        }
        
    }

    bool LoopClosure::IsKeyframeInWaitingList()
    {
        // Return True if there are keyframes waiting to be check for loop

        std::unique_lock<std::mutex> lck(list_mutex_);
        return not(waitlist_keyframes_.empty());
    }

    void LoopClosure::SetCurrentKeyframe()
    {
        // Extract most waited keyframe from waiting list
        
        std::unique_lock<std::mutex> lck(list_mutex_);
        current_keyframe_ = waitlist_keyframes_.front();
        waitlist_keyframes_.pop_front();
    }

    void LoopClosure::AddToProcessedKeyframes()
    {
        /* Adds the current keyframe with extracted features
         * to the processed keyframes collection */
        
        std::unique_lock<std::mutex> lck(database_mutex_);
        processed_keyframes_[current_keyframe_->keyframe_id_] = current_keyframe_;
        last_keyframe_ = current_keyframe_;
    }

    bool LoopClosure::LoopKeyframeCandidate()
    {
        /* Uses comparision of deep feature vector representaion of whole image of 
         * current frame and all previously processed keyframes to find a potential 
         * loop candidate for current frame. */
        
        std::unique_lock<std::mutex> lck(database_mutex_);

        candid_loop_keyframe_ = nullptr;

        float max_similarity{0};
        unsigned long max_similarity_kf_id{0};
        int num_potential_candidate{0};

        for(auto iter{processed_keyframes_.begin()}; iter != processed_keyframes_.end(); iter++)
        {
            // Ignore recent keyframes to current keyframes
            if(current_keyframe_->keyframe_id_ - iter->first < 20)
            {
                continue;
            }

            // Calculate similarty
            float similarity = SimilarityScore(current_keyframe_->representation_vec_,
                                                 iter->second->representation_vec_);

            if(similarity > max_similarity)
            {
                max_similarity = similarity;
                max_similarity_kf_id = iter->first;
            }
            if(similarity > potential_loop_weak_threshold_)
            {
                ++num_potential_candidate;
            }
        }

        // If visualization enable, plot score for keyframe
        if(viewer_)
        {
            viewer_->Plot("plots/loop_deep_score", 
                 max_similarity, 
                 current_keyframe_->keyframe_id_);
            
        }

        /* If max similarity does not pass strong threshold for score or 
         * there are to many similar keyframe with high matching score, 
         * it means no potential loop candidate */
        if((max_similarity < potential_loop_strong_threshold_) or (num_potential_candidate > max_num_weak_threshold_))
        {
            return false;
        }

        // Set candidate with potential for loop
        candid_loop_keyframe_ = processed_keyframes_[max_similarity_kf_id];
        return true;
    }

    bool LoopClosure::KeypointMatchWithLoopCandid()
    {
        /* Match keypoint in current keyframe and the candidate
         * keyframe for loop. Return true if enough match was found. */
        
        KeypointMatches_.clear();

        // Match keypoints 
        std::vector<cv::DMatch> matches;
        matcher_->match(candid_loop_keyframe_->descriptor_, 
                        current_keyframe_->descriptor_, 
                        matches);

        // Find match with smallest distance between keypoints descriptors
        auto min_elm = std::min_element(matches.begin(), matches.end());
        
        // Threshold for reject bad matches
        double distance_threshold = std::max(2 * min_elm->distance, 30.0f);
        
        // Discard bad matches
        for(auto &match: matches)
        {
            if(match.distance > distance_threshold)
            {
                continue;
            }

            // index on corresponding features
            std::pair<size_t, size_t> corres{
                candid_loop_keyframe_->desc_feat_indx_[match.queryIdx],
                current_keyframe_->desc_feat_indx_[match.trainIdx]};
            KeypointMatches_.insert(corres);
        }

        if(KeypointMatches_.size() >= min_num_acceptable_keypoint_match_)
        {
            return true;
        }

        return false;
    }

    bool LoopClosure::CalculatePose()
    {
        /* Use the left image's keypoint features of the current keyframe and 
         * the landmarks associated with features in the left image of the keyframes
         * that form a loop with the current keyframe to calculate the correct pose
         * of the current keyframe.
         */


        // Prepare 3D point and 2D keypoint feature
        std::vector<cv::Point3f> points3d_cand;
        std::vector<cv::Point2f> points2d_curr;
        for(auto iter{KeypointMatches_.begin()}; iter != KeypointMatches_.end();)
        {   
            // Feature in current keyframe of loop closure pipeline
            Feature::Ptr feat_curr = current_keyframe_->feature_left_[iter->second];

            // Featue in candidate keyframe for loop
            Feature::Ptr feat_cand = candid_loop_keyframe_->feature_left_[iter->first];

            // map point that feature in candidate keyframe refer to
            MapPoint::Ptr mp = feat_cand->map_point_.lock();

            /* Check if there is 3D map point associated with feature, collect
             * map point and corresponding feature in current keyframe */
            if(mp)
            {
                Eigen::Vector3d mp_pos = mp->Pos();
                points3d_cand.push_back(cv::Point3f(mp_pos(0), mp_pos(1), mp_pos(2)));
                points2d_curr.push_back(feat_curr->position_.pt);

                iter++;
            }
            else
            {
                // Remove from matches
                iter = KeypointMatches_.erase(iter);
            }
        }

        if(points3d_cand.size() < min_num_acceptable_keypoint_match_)
        {
            return false;
        }

        /* Calculate pose of current keyframe left camera 
         * in world coodinate with PnP Ransac */
        cv::Mat rot_vec, t_vec, R_cv, K;
        cv::eigen2cv(cam_left_->K(), K);
        std::vector<int> inliers; 
        try
        {
            cv::solvePnPRansac(points3d_cand, points2d_curr, K, cv::Mat(),
                               rot_vec, t_vec, false, 100, 5.991, 0.99, inliers);
        }
        catch(...)
        {
            return false;        
        }

        // Convert to Eigen rotation matrix and translation vector
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        cv::Rodrigues(rot_vec, R_cv);
        cv::cv2eigen(R_cv, R);
        cv::cv2eigen(t_vec, t);

        if(inliers.size() < min_num_acceptable_keypoint_match_)
        {
            return false;
        }
        
        // Corrected pose of current frame
        current_frame_corrected_pose_ = cam_left_->pose_.inverse() * Sophus::SE3d(R, t);
        
        // Transformation from loop keyframe to current keyframe new pose
        current_keyframe_->loop_relative_pose_ = current_frame_corrected_pose_ * 
                                                 candid_loop_keyframe_->Pose().inverse();

        /* If new calculated pose of current keyframe is similar to 
         * previous value of pose, there is no need for path optimization
         * and updating pose of other keyframes */ 
        double pose_diff = (current_keyframe_->Pose() *
                                current_frame_corrected_pose_.inverse()).log().norm();

        /* If new calculated and old value pose difference is high, it is potential
         * for miss calculation. Ignore it. */
        if(pose_diff > max_pose_differnece_between_old_new_)
        {
            return false;
        }

        if(pose_diff > min_pose_differnece_between_old_new_)
        {
            need_correction_ = true;
        }
        else
        {
            need_correction_ = false;
        }


        return true;
    }

    void LoopClosure::LocalFusion()
    {
        /* After detecting a loop, recalculates active keyframes'poses 
         * and active landmarks' positions by leveraging new pose
         * of the keyframe obtained by loop detection. Also, updates
         * current frame in frontend pipeline. 
         */
        
        // Lock for avoiding conflict with frontend
        std::unique_lock<std::mutex> lck(loop_closure_upd_);

        // Calculate correct pose of active keyframes after loop detection 
        std::unordered_map<unsigned long, Sophus::SE3d> corrected_poses;
        corrected_poses[current_keyframe_->keyframe_id_] = current_frame_corrected_pose_;
        for(auto &kf_i: map_->GetActiveKeyFrames())
        {
            unsigned long kf_id_i = kf_i.first;
            if(current_keyframe_->keyframe_id_ == kf_id_i)
            {
                continue;
            }

            // Transformation from Current keyframe to ative keyframe i
            Sophus::SE3d T_i_c = kf_i.second->Pose() * (current_keyframe_->Pose().inverse()); 
            // Corrected transformation from World to active keyframe i
            Sophus::SE3d T_i_w = T_i_c * current_frame_corrected_pose_; 

            corrected_poses[kf_id_i] = T_i_w;
        }

        // Calculate correct position of landmarks after loop detection
        for(auto &mappoint_i: map_->GetActiveMapPoints())
        {
            MapPoint::Ptr mp_i = mappoint_i.second;

            /* Find the oldest keyframe that has a 
             * keypoint feature that point to the map point */
            Frame::Ptr kf_old{nullptr};
            for(auto &feat_i: mp_i->GetObs())
            {
                Feature::Ptr feat_first = feat_i.lock();
                if(feat_first)
                {
                    if(feat_first->map_point_.lock())
                    {
                        kf_old = feat_first->frame_.lock();
                        if(corrected_poses.find(kf_old->keyframe_id_) != corrected_poses.end())
                        {
                            break;
                        }
                    }
                }

                kf_old.reset();
            }

            assert(kf_old != nullptr);

            // Point in stereo system coordinate of 'kf_old' keyframe
            Eigen::Vector3d pos_s =  kf_old->Pose() * mp_i->Pos();
            // Find new position of point in world coordinate with corrected pose
            Eigen::Vector3d pos_w = corrected_poses.at(kf_old->keyframe_id_).inverse() * pos_s;
            // Set new postion of point
            mp_i->SetPos(pos_w);
        }

        // Update pose of last frame that was fed to SLAM pipeline in frontend
        Frame::Ptr frontend_lf = frontend_.lock()->GetLastFrame();
        Sophus::SE3d T_f_c = frontend_lf->Pose() * (current_keyframe_->Pose().inverse());
        bool is_front_lf_active_kf{false};

        // Replace active keyframes old pose with new pose
        for(auto &kf_i: map_->GetActiveKeyFrames())
        {
            kf_i.second->SetPose(corrected_poses.at(kf_i.first));
            
            if(frontend_lf->keyframe_id_ == kf_i.first)
            {
                is_front_lf_active_kf = true;
            }
        }
        
        // Continue - update last frame pose
        if(is_front_lf_active_kf == false)
        {
            Sophus::SE3d T_f_w = T_f_c * current_frame_corrected_pose_; 
            frontend_lf->SetPose(T_f_w);
        }

        /* Replace landmarks associated with current keyframe with map points 
         * associated with the other keyframe involved in detected loop */
        for(auto &pair_i: KeypointMatches_)
        {
            // Index of corresponding keypoint features matches
            size_t cur_kf_feat_idx{pair_i.second};
            size_t loop_kf_feat_idx{pair_i.first};

            // Map point associated with feature in loop keyframe
            MapPoint::Ptr mp_loop = candid_loop_keyframe_->
                                        feature_left_[loop_kf_feat_idx]->
                                            map_point_.lock();

            if(mp_loop)
            {
                // Map point associated with feature in current keyframe
                MapPoint::Ptr mp_curr = current_keyframe_->
                                        feature_left_[cur_kf_feat_idx]->
                                            map_point_.lock();
                
                /* Replace map point linked to feature in current frame 
                 * with map point of the feature in the other keyframe */
                if(mp_curr)
                {
                    for(auto &observation_i: mp_curr->GetObs())
                    {
                        Feature::Ptr obs_i = observation_i.lock();
                        if(obs_i)
                        {
                            mp_loop->AddObservation(obs_i);
                            obs_i->map_point_ = mp_loop;
                        }
                    }
                    // Remove the current keyframe point
                    map_->RemoveLandmark(mp_curr);
                }
                else
                {
                    current_keyframe_->
                                        feature_left_[cur_kf_feat_idx]->
                                            map_point_ = mp_loop;
                }

            }

        }

        if(viewer_)
        {
            viewer_->LogInfoMKF(
                "Loop   : Local path fusion done.",
                current_keyframe_->keyframe_id_, "loopclosure");
        }

    }

    void LoopClosure::LoopClosureUpdate()
    {
        /* This function updates the positions of active landmarks, the poses 
         * of active keyframes and frontend's last processed frame after a loop is  
         * detected. It aligns the positions with the new pose of the current 
         * keyframe within the loop closure pipeline, resolving any conflicts 
         * between the frontend and backend with the newly detected loop. */

        /* If current keyframe pose and its new calulated value similar, 
         * no correction required */
        if(need_correction_ == false)
        {
            return;
        }    
        
        // Pause Backend optimization during loop closure correction
        {
            Backend::Ptr bk = backend_.lock();
            if(bk)
            {
                if(bk->IsRunning())
                {
                    bk->PauseRequest();

                    while(not bk->IsPaused())
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }

                    if(viewer_)
                    {
                        viewer_->LogInfoMKF("Backend: Paused ", 
                                            current_keyframe_->keyframe_id_,
                                            "backend");
                    }
                }
            }
        }
        
        /* Resolve conflict of frontend/backend with
         * current keyframe after detecting loop */
        LocalFusion();
        
        // Resume Backend optimization
        {
            Backend::Ptr bk = backend_.lock();
            if(bk)
            {
                if(bk->IsRunning())
                {
                    bk->Resume();
                }
            }
        }

    }

    void LoopClosure::PoseGraphOptimization()
    {
        /* Pose graph optimization of all keyframes and all map points
         * (landmarks) to further use information of closed loops to improve
         * parameters.
         * 
         * For more information see Chapter 10 (loop closure) of
         * "Introduction to Visual SLAM: From Theory to Practice" 
         * by Xiang Gao and Tao Zhang.
         * */
        
        if(viewer_)
        {
            viewer_->LogInfoMKF(
                "Loop   : Pose Graph Optimization started ... (Takes seconds to minutes)",
                current_keyframe_->keyframe_id_, "loopclosure");
        }

        /* A 6*6 block is used because the optimized parameters represent the
         * keyframe's pose as a 6D vector (Lie algebra: 3 for translation
         * + 3 for rotation), and the observations are again 6D pose vector. */
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;

        // Linear solver for solving linear subproblem during non-linear optimization
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        /* Levenberg-Marquardt algorithm, which is used for nonlinear 
         * least squares optimization Memory deallocation handled by
         * optimizer */
        g2o::OptimizationAlgorithmLevenberg* solver = 
            new g2o::OptimizationAlgorithmLevenberg(
                std::make_unique<BlockSolverType>(
                    std::make_unique<LinearSolverType>()));

        // Optimizer
        g2o::SparseOptimizer optimizer; 
        optimizer.setAlgorithm(solver); 

        
        // Get all keyframes of map
        Map::KeyframesType all_keyframes = map_->GetAllKeyFrames();

        // Get all landmarks
        Map::LandmarksType all_landmarks = map_->GetAllMapPoints();

        /* Vertices for pose optimization graph is all keyframes'
         * poses */ 
        std::map<unsigned long, VertexPose *> vertices; 
        for(auto &id_keyframe_pair: all_keyframes)
        {
            unsigned long kfid_i = id_keyframe_pair.first;
            auto kf_i = id_keyframe_pair.second; 

            VertexPose *vertex_i = new VertexPose();
            vertex_i->setId(kf_i->keyframe_id_);
            vertex_i->setEstimate(kf_i->Pose());
            vertex_i->setMarginalized(false);

            /* During pose graph optimization, pose of first keyframe,
             * is set as constant */
            if( kfid_i == 0UL)
            {
                vertex_i->setFixed(true);
            }

            // Add vertex to graph
            optimizer.addVertex(vertex_i);
            vertices[kfid_i] = vertex_i;
        }

        // Add edges to graph
        int indx{0};
        std::map<int, EdgePoseGraph *> edges;
        for(auto &id_keyframe_pair: all_keyframes)
        {
            unsigned long kfid_i = id_keyframe_pair.first;
            auto kf_i = id_keyframe_pair.second; 

            // Edge between two consequitive keyframe in time
            Frame::Ptr kf_i_prev = kf_i->prev_keyframe_.lock();
            if(kf_i_prev)
            {
                EdgePoseGraph *edge_i = new EdgePoseGraph();
                edge_i->setId(indx);
                edge_i->setVertex(0, vertices.at(kfid_i));
                edge_i->setVertex(1, vertices.at(kf_i_prev->keyframe_id_));
                edge_i->setMeasurement(kf_i->relative_pose_pkf_);
                edge_i->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
                
                // Add edge to graph
                optimizer.addEdge(edge_i);
                
                edges.insert({indx, edge_i});
                indx++;
            }

            // Edge between two keyframe involved in detected loop
            Frame::Ptr loop_kf_i = kf_i->loop_keyframe_.lock();
            if(loop_kf_i)
            {
                EdgePoseGraph *edge_i = new EdgePoseGraph();
                edge_i->setId(indx);
                edge_i->setVertex(0, vertices.at(kfid_i));
                edge_i->setVertex(1, vertices.at(loop_kf_i->keyframe_id_));
                edge_i->setMeasurement(kf_i->loop_relative_pose_);
                edge_i->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
                
                // Add edge to graph
                optimizer.addEdge(edge_i);
                
                edges.insert({indx, edge_i});
                indx++;
            }
        }

        // Perform non-linear least square optimization for pose graph
        optimizer.initializeOptimization();
        optimizer.optimize(22);

        // Calculate new posisition of landmarks
        for(auto &id_mappoint_i: all_landmarks)
        {
            MapPoint::Ptr mp_i = id_mappoint_i.second;
            assert(mp_i != nullptr);

            // First feature that observe the mappoint 
            Feature::Ptr feat = mp_i->first_valid_obs_.lock();
            assert(feat != nullptr);
            
            // Keyframe of feature
            Frame::Ptr kf_feat = feat->frame_.lock();
            if(vertices.find(kf_feat->keyframe_id_) == vertices.end())
            {
                continue;
            }    
            
            // Point in stereo system coordinate of 'kf_feat' keyframe
            Eigen::Vector3d pos_s =  kf_feat->Pose() * mp_i->Pos();
            // Find new position of point in world coordinate with corrected pose
            Eigen::Vector3d pos_w = (
                        vertices.at(kf_feat->keyframe_id_)->estimate()).inverse() * pos_s;
            // Set new postion of point
            mp_i->SetPos(pos_w);
        }
        
        // Set the Keyframes' new poses
        for (auto &vertex_i: vertices) 
        {
            all_keyframes.at(vertex_i.first)->SetPose(vertex_i.second->estimate());
        }

        if(viewer_)
        {
            viewer_->LogInfoMKF(
                "Loop   : Pose Graph Optimization is done.",
                current_keyframe_->keyframe_id_, "loopclosure");
        }

    }

    void LoopClosure::LoopClosurePipeline()
    {
        /* Running in separate thread, constantly check new keyframes,
         * if a loop detected, call loop closure pipeline to 
         * correct camera poses and landmarks locations */

        while(loopclosure_running_.load())
        {
            /* If there is no keyfram in waiting list,
             * wait and try again */
            if(not(IsKeyframeInWaitingList()))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // Choose a keyframe from waiting list
            SetCurrentKeyframe();
            
            // Calculate deep feature vector representation for left image
            ExtractFeatureVec(current_keyframe_);
            
            // Extract keypoint features descriptor
            ExtractKeypointsDescriptor(current_keyframe_);

            bool loop_detected{false};
            candid_loop_keyframe_.reset();

            /* If a potential loop is detected, proceed to the
             * next steps to check for the existence of a loop */
            if(LoopKeyframeCandidate())
            {
                /* If a good mathcing between current keyframe and 
                 * the potential keyframe candidate exist, proceed */
                if(KeypointMatchWithLoopCandid())
                {
                    /* Calculate correct pose of current keyframe by using   
                     * information of loop. If number of inliers during calculating
                     * pose is acceptable, a loop is confirmed */
                    if(CalculatePose())
                    {
                        // Set a loop detected
                        loop_detected = true;
                        // Link current keyframe to loop keyframe
                        current_keyframe_->loop_keyframe_ = candid_loop_keyframe_;
                        
                        if(viewer_)
                        {
                            viewer_->LogInfoMKF(
                                "Loop   : Loop Detected between keyframes " +
                                 std::to_string(current_keyframe_->keyframe_id_) + 
                                 std::string("/") + std::to_string(candid_loop_keyframe_->keyframe_id_),
                                current_keyframe_->keyframe_id_, "loopclosure");
                        }

                        /* Refine pose and position of keyframes and landmarks locally
                         * after loop detection */
                        LoopClosureUpdate();

                        // Update last closed keyframe
                        last_closed_keyframe_ = current_keyframe_;
                    }
                }
            }

            /* Add the processed keyframe with extracted deep feature and descriptors
             * for comparing future keyframes with it */
            if(loop_detected == false)
            {
                AddToProcessedKeyframes();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        // Global pose graph optimization
        if(global_pose_graph_optimization_ == true)
        {
            PoseGraphOptimization();
        }
    }

    void LoopClosure::Stop() 
    {
        // Close loop closure optimization
        loopclosure_running_.store(false);
        loopclosure_thread_.join();

        if(viewer_)
        {
            viewer_->LogInfoMKF("Loop   : Stopped ", 
                                current_keyframe_->keyframe_id_, 
                                "loopclosure");
        }
    }

}