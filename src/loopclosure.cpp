#include<StereoVisionSLAM/loopclosure.h>
#include<StereoVisionSLAM/visual_odometry.h>
#include<StereoVisionSLAM/slamexception.h>
#include "StereoVisionSLAM/config.h"
#include <opencv2/features2d.hpp>

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

    LoopClosure::LoopClosure()
    {
        // Set hyperparameters
        keyframes_to_ignore_after_loop_ = Config::Get<int>("keyframes_to_ignore_after_loop");
        potential_loop_weak_threshold_ = Config::Get<float>("potential_loop_weak_threshold");
        potential_loop_strong_threshold_ = Config::Get<float>("potential_loop_strong_threshold");
        max_num_weak_threshold_ = Config::Get<int>("max_num_weak_threshold");
        min_num_acceptable_keypoint_match_ = Config::Get<int>("min_num_acceptable_keypoint_match");

        // Initialize Deep Neural Network for feature extraction from images
        InitialFeatureExtractorNetwork();

        // Initialize backend optimization in a seprate thread
        loopclosure_running_.store(true);
        loopclosure_thread_ = std::thread(std::bind(&LoopClosure::LoopClosureLoop, this));
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
        /* 
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

        // Calculate pose of current keyframe in world coodinate with PnP Ransac
        try
        {
    
        }
        catch(...)
        {
            return false;        
        }
        



        return true;
    }

    void LoopClosure::LoopClosureLoop()
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
                usleep(1000);
                continue;
            }

            // Choose a keyframe from waiting list
            SetCurrentKeyframe();
            
            // Calculate deep feature vector representation for left image
            ExtractFeatureVec(current_keyframe_);
            
            // Extract keypoint features descriptor
            ExtractKeypointsDescriptor(current_keyframe_);

            bool loop_detected{false};
            
            /* If a potential loop is detected, proceed to the
             * next steps to check for the existence of a loop */
            if(LoopKeyframeCandidate())
            {
                /* If a good mathcing between current keyframe and 
                 * the potential keyframe candidate exist, proceed */
                if(KeypointMatchWithLoopCandid())
                {
                    if(CalculatePose())
                    {
                        // Set a loop detected
                        loop_detected = true;
                        last_closed_keyframe_ = current_keyframe_;

                        //
                        std::cout << current_keyframe_->keyframe_id_ << ", " << candid_loop_keyframe_->keyframe_id_ << std::endl;
                    }
                }
            }




            /* Add processed keyframe with extracted features for comparing
             * future keyframe with it */
            if(loop_detected == false)
            {
                AddToProcessedKeyframes();
            }

            usleep(1000);
        }
    }

    void LoopClosure::Stop() 
    {
        // Close loop closure optimization
        loopclosure_running_.store(false);
        loopclosure_thread_.join();
    }

}