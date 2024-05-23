#include "StereoVisionSLAM/frontend.h"
#include "StereoVisionSLAM/algorithm.h"
#include "StereoVisionSLAM/config.h"
#include "StereoVisionSLAM/g2o_types.h"

namespace slam
{

    Frontend::Frontend()
    {
        // Initialize hyper-parameters of frontend
        num_features_ = Config::Get<int>("num_features");
        num_features_init_ = Config::Get<int>("num_features_init");
        num_features_tracking_ = Config::Get<int>("num_features_tracking");

        // Initialize keypoint detector
        gftt_ = cv::GFTTDetector::create(num_features_, 0.01, 20);
    }

    int Frontend::DetectFeatures()
    {
        // Detect keypoint features in current frame left image

        // If previously tracked keypoint are exist for the frame, 
        // masked them to explore new areas and ekypoints
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
        for(Feature::Ptr &feat_i: current_frame_->feature_left_)
        {
            cv::rectangle(mask, feat_i->position_.pt - cv::Point2f(10,10),
                          feat_i->position_.pt + cv::Point2f(10,10), 0, cv::FILLED);
        }

        // detect keypoints
        std::vector<cv::KeyPoint> keypoints;
        gftt_->detect(current_frame_->left_img_, keypoints, mask);
        int cnt_detected{keypoints.size()};
        
        // Add new keypoint to features of left image 
        for(cv::KeyPoint &kp:keypoints)
        {
            current_frame_->feature_left_.push_back(
                std::make_shared<Feature>(current_frame_, kp));
        }

        std::cout << "Detect " << cnt_detected << " new features" << std::endl;
        return cnt_detected;
    }

    int Frontend::FindFeaturesInRight()
    {
        /* Use LK flow to track keypoint features from the  
          left image in the right image */
        
        std::vector<cv::Point2f> kps_left, kps_right;
        
        /* Prepare keypoint features in the left image. If there 
          are associated 3D points in the map with them, use their 
          projections into the right image as the initial guess */
        for(auto &feat_i: current_frame_->feature_left_)
        {
            kps_left.push_back(feat_i->position_.pt);

            // Map point associated to keypoint feature
            MapPoint::Ptr mp = feat_i->map_point_.lock();
            if(mp)
            {
                // Project map point to right image
                Eigen::Vector2d px_right = camera_right_->world2pixel(
                    mp->pos_, current_frame_->Pose());
                
                kps_right.push_back(cv::Point2f(px_right[0], px_right[1]));
            }
            else
            {
                kps_right.push_back(feat_i->position_.pt);
            }
        }

        // LK sparse optical flow to track keypoint features
        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(
            current_frame_->left_img_, current_frame_->right_img_, kps_left,
            kps_right, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        // Discard outlier tracked point in right image
        int num_good_pts{0};
        for(size_t i{0}; i < status.size(); ++i)
        {
            if(status[i])
            {
                cv::KeyPoint kp_i(kps_right[i], 7);
                Feature::Ptr feat = std::make_shared<Feature>(current_frame_, kp_i);
                feat->is_on_left_image_ = false;
                current_frame_->feature_right_.push_back(feat);
                num_good_pts++;
            }
            else
            {
                current_frame_->feature_right_.push_back(nullptr);
            }
        }

        std::cout << "Find " << num_good_pts << " in the right image." << std::endl;
        return num_good_pts;
    }

    bool Frontend::BuildInitMap()
    {
        // Generate initial map from triangulation of matching points
        // in the left and right camera image pair

        // Retrieve left and right camera poses for triangulation
        std::vector<Sophus::SE3d> poses{camera_left_->pose(), camera_right_->pose()};

        int cnt_init_landmarks{0};

        // For each matching point pair in left and right image
        // perform triangulation to ontain 3d point in stereo system coordinate
        for(size_t i{0}; i < current_frame_->feature_left_.size(); ++i)
        {
            // Ignore the left feature if does not have a match in right image
            if(current_frame_->feature_right_[i] == nullptr)
            {
                continue;
            }

            // Convert keypoint features from camera pixel  
            // into normal plane coordinate of each camera
            std::vector<Eigen::Vector3d> points{
                camera_left_->pixel2camera(toVec2(current_frame_->feature_left_[i]->position_.pt))
                ,
                camera_right_->pixel2camera(toVec2(current_frame_->feature_right_[i]->position_.pt))
            };

            // Perfrom triangulation to obtain 3d location of point in 
            // stereo system coordinate
            Eigen::Vector3d pworld = Eigen::Vector3d::Zero();
            if(triangulation(poses, points, pworld) and (pworld[2] > 0))
            {
                // Create map point
                MapPoint::Ptr new_map_point = MapPoint::CreateNewMappoint();
                // World coordinate system (map coordinate system) is euqal
                // to stereo coordinate system at initialization
                new_map_point->SetPos(pworld);
                
                // Link features and the new map point
                new_map_point->AddObservation(current_frame_->feature_left_[i]);
                new_map_point->AddObservation(current_frame_->feature_right_[i]);
                current_frame_->feature_left_[i]->map_point_ = new_map_point;
                current_frame_->feature_right_[i]->map_point_ = new_map_point;

                // Add point to map
                map_->InsertMapPoint(new_map_point);

                cnt_init_landmarks++;
            }
        }

        // Set first frame as a keyframe
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);

        // **
        // backend_->UpdateMap();

        std::cout << "Initial map created with " << cnt_init_landmarks << " map points" << std::endl;

        return true;
    }

    bool Frontend::StereoInit()
    {
        // Try to initialize frontend with stereo images

        // Detect keypoint features in left image
        int num_features_left = DetectFeatures();
        
        // Find corresponding matches for the detected keypoint features
        // in the right image
        int num_good_features = FindFeaturesInRight();
        
        if(num_good_features < num_features_init_)
        {
            return false;
        }

        // itialize map
        bool build_map_success = BuildInitMap();
        if(build_map_success)
        {
            status_ = FrontendStatus::TRACKING_GOOD;
            
            if(viewer_)
            {
                viewer_->AddCurrentFrame(current_frame_);
                viewer_->UpdateMap();
            }
            return true;
        }

        return false;
    }

    int Frontend::TriangulateNewPoints()
    {
        // Triangulate corresponding keypoint features in 
        // left and right images of current frame

        // Retrieve left and right camera poses for triangulation
        std::vector<Sophus::SE3d> poses{camera_left_->pose(), camera_right_->pose()};

        // Transformation from current frame coordinate system to
        // world coordinate system (map coordinate system)
        Sophus::SE3d current_pose_Twc = current_frame_->Pose().inverse();

        int cnt_triangulated_pts{0};
        
        // Triangulate each corresponding feature pair in 
        // current frame left and right images
        for(size_t i{0}; i < current_frame_->feature_left_.size(); i++)
        {
            // Ignore ith feature in the left image if there is no corresponding   
            // feature in the right image or if a 3D point is already 
            // associated with it in the map.
            if(current_frame_->feature_left_[i]->map_point_.expired() and 
                current_frame_->feature_right_[i] != nullptr)
            {
                // Convert keypoint features from camera pixel  
                // into normal plane coordinate of each camera
                std::vector<Eigen::Vector3d> points{
                    camera_left_->pixel2camera(toVec2(current_frame_->feature_left_[i]->position_.pt))
                    ,
                    camera_right_->pixel2camera(toVec2(current_frame_->feature_right_[i]->position_.pt))
                };

                // Perfrom triangulation to obtain 3d location of point in 
                // stereo system coordinate
                Eigen::Vector3d pworld = Eigen::Vector3d::Zero();
                if(triangulation(poses, points, pworld) and (pworld[2] > 0))
                {   
                    // Create a new map point by result of trangulation
                    MapPoint::Ptr new_map_point = MapPoint::CreateNewMappoint();

                    // Transform 3d triangulated point from stereo system
                    // coordinate to world coordinate system (map coordinate)
                    pworld = current_pose_Twc * pworld;
                    new_map_point->SetPos(pworld);
                    
                    // Link features and the new map point
                    new_map_point->AddObservation(current_frame_->feature_left_[i]);
                    new_map_point->AddObservation(current_frame_->feature_right_[i]);
                    current_frame_->feature_left_[i]->map_point_ = new_map_point;
                    current_frame_->feature_right_[i]->map_point_ = new_map_point;

                    // add point to map
                    map_->InsertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }

        std::cout << "new landmarks: " << cnt_triangulated_pts << std::endl;
        return cnt_triangulated_pts;
    }

    int Frontend::TrackLastFrame()
    {
        // Track last frame left image features in current frame
        // left image

        // Retrieve keypoint features in the left image of the last frame.
        // If 3D points are associated with these features, use their projections
        // into the current frame as initial guesses for optical flow.
        std::vector<cv::Point2f> kps_last, kps_current;
        for(Feature::Ptr &feat_i: last_frame_->feature_left_)
        {
            kps_last.push_back(feat_i->position_.pt);
            
            // Associated map point to i th left feature in last frame
            MapPoint::Ptr mp = feat_i->map_point_.lock();
            if(mp)
            {   
                // project to left image of current frame
                Eigen::Vector2d px = camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
                kps_current.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                kps_current.push_back(feat_i->position_.pt);
            }
        }
        
        // Track feature of last frame in current
        // frame by using optical flow
        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(
            last_frame_->left_img_, current_frame_->left_img_, kps_last,
            kps_current, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        // Just keep good tracked features
        int num_good_pts{0};
        for(size_t i{0}; i < status.size(); ++i)
        {
            if(status[i])
            {
                // Create feature and add to current frame left image features
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr feature = std::make_shared<Feature>(current_frame_, kp);
                current_frame_->feature_left_.push_back(feature);
                // link features to its 3D point in world coordinate (map point)
                feature->map_point_ = last_frame_->feature_left_[i]->map_point_;
                num_good_pts++;
            }
        }

        std::cout << "Find " << num_good_pts << " in the last image." << std::endl;
        return num_good_pts;
    }

    int Frontend::EstimateCurrentPose()
    {
        /*
        Utilizes tracked keypoint features from the last frame to estimate
        the pose of the current frame in the world coordinate system
        (map coordinate). It optimizes by minimizing the projection error
        of landmarks and their corresponding tracked features. The optimization
        process employs a coarse-to-fine hierarchical nonlinear optimization
        approach, incorporating outlier detection in tracked points.
        */

        /* A 6*3 block is used because the optimized parameters represent the
         current frame's  pose as a 6D vector (Lie algebra: 3 for translation
         + 3 for rotation), and the observations are 3D landmarks (map points).*/
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        
        // Linear solver for solving linear subproblem during no-linear optimization
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        /* Levenberg-Marquardt algorithm, which is used for nonlinear least squares optimization
         Memory deallocation handled by optimizer*/
        g2o::OptimizationAlgorithmLevenberg* solver = 
            new g2o::OptimizationAlgorithmLevenberg(
                std::make_unique<BlockSolverType>(
                    std::make_unique<LinearSolverType>()));

        // Optimizer
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        /* Optimization graph for obtaining the pose of the current frame
           contains a single vertex */
        VertexPose *vertex_pose = new VertexPose(); // Memory deallocation handled by optimizer
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.addVertex(vertex_pose);

        // K: left camera intrinsic matrix
        Eigen::Matrix3d K = camera_left_->K();

        /* In the optimization graph, each keypoint feature in current frame
           left camera with a corresponding 3D point in the map defines an edge*/
        int index{1};
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<Feature::Ptr> features;
        for (size_t i{0}; i < current_frame_->feature_left_.size(); ++i)
        {
            /* Retreive 3D map point (landmark) associated with the 
               i'th keypoint feature in current frame's left camera */
            MapPoint::Ptr mp = current_frame_->feature_left_[i]->map_point_.lock();
            if(mp)
            {
                features.push_back(current_frame_->feature_left_[i]);
                
                // Create a new edge, memory deallocation handled by optimizer
                EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
                edge->setId(index);
                
                // Connect edge to vertex in graph
                edge->setVertex(0, vertex_pose);
                
                // Set the measurement (observed 2D keypoint point in left image)
                edge->setMeasurement(toVec2(current_frame_->feature_left_[i]->position_.pt));

                /* Set information matrix, it is the inverse of the covariance 
                 and indicates the confidence in the measurement*/
                edge->setInformation(Eigen::Matrix2d::Identity());

                // Kernel to decrease effect of outliers
                edge->setRobustKernel(new g2o::RobustKernelHuber);

                edges.push_back(edge);
                // Add edge tp graph
                optimizer.addEdge(edge);

                index++;
            }
        }
        

        // Numner of outliers
        int cnt_outlier{0};
        // threshold for determining outlier
        const double chi2_th{5.991};

        /* Estimate the pose and determine the outliers Perform
         the whole optimization 4 times, each time detected 
         outliers will result in a potential better solution*/ 
        for(int iteration{0}; iteration < 4; ++iteration)
        {
            // initialize parameters
            vertex_pose->setEstimate(current_frame_->Pose());
            // Prform optimization
            optimizer.initializeOptimization();
            optimizer.optimize(10);

            cnt_outlier = 0;

            /* Count the outliers and accoding to outlier status
             of a edge, assign different level prority to edges
             for coarse-to-fine heirarchical optimization*/
            for(size_t i{0}; i < edges.size(); ++i)
            {
                auto e =edges[i];
                if (features[i]->outlier_)
                {
                    e->computeError();
                }

                // Update outlier status of feature/edge
                if(e->chi2() > chi2_th)
                {
                    features[i]->outlier_ = true;
                    // For coarse-to-fine heirarchical optimization
                    e->setLevel(1);
                    cnt_outlier++;
                }
                else
                {
                    features[i]->outlier_ = false;
                    // For coarse-to-fine heirarchical optimization
                    e->setLevel(0);
                }

                if(iteration == 2)
                {
                    /* Remove robust kernel for more aggressive
                     optimization for the last iteration*/
                    e->setRobustKernel(nullptr);
                }

            }

        }

        std::cout << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"  << features.size() - cnt_outlier;
        
        // Set pose of current frame
        current_frame_->SetPose(vertex_pose->estimate());

        std::cout << "Current Pose = \n" << current_frame_->Pose().matrix() << std::endl;

        /* Remove links between landmarks (3D points in the map) 
         and features that are outliers*/
        for (auto &feat : features) 
        {
            if (feat->outlier_) 
            {
                feat->map_point_.reset();
                feat->outlier_ = false;  // maybe we can still use it in future
            }
        }

        // return number of inliers features in left camera of current image
        return features.size() - cnt_outlier;

    }

    void Frontend::SetObservationsForKeyFrame()
    {
        /* Link a landmark (3d point in map) to its corresponding keypoint
           feature in current frame left camera */
        for(auto &feat_i: current_frame_->feature_left_)
        {
            // Map point (landmark) that keypoint feature refers to
            MapPoint::Ptr mp = feat_i->map_point_.lock();
            if(mp)
            {
                // link landmark to keypoint feature
                mp->AddObservation(feat_i);
            }
        } 
    }

    bool Frontend::InsertKeyframe()
    {   
        /* Determine if the current frame is a keyframe by examining the number 
          of inlier tracked keypoint features from the left image of the last 
          frame to the left image of the current frame. If it is a keyframe, 
          detect new keypoint features to improve quality of feature set
          and find their 3D map point correspondences by using triangulation 
          with the right image of the current frame. */

        /* If current frame does have enough good tracked keypoint
           from last frame, it's not a keyframe */
        if(tracking_inliers_ >= num_features_needed_for_keyframe_)
        {
            return false;
        }

        // Set current frame as keyframe and add it to map
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);

        std::cout << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_ << std::endl;
 
        SetObservationsForKeyFrame();

        // Detect new keypoint features in current frame left image
        DetectFeatures();
        // Track in right image
        FindFeaturesInRight();
        // Triangulate track fetures in left and righ images 
        TriangulateNewPoints();

        // update backend because we have a new keyframe
        // backend_->UpdateMap();
        
        if (viewer_)
        {
            viewer_->UpdateMap();
        }

        return true;
    }

    bool Frontend::Track()
    {
        /* Tracks the current frame using the last frame, updates 
           the current frame pose, tracking status, and checks for keyframes */

        /* Assume relative pose between the last frame and current frame
         equals to relative pose between last frame and its previous frame,
         use this assumption to obtain an initial value for current frame pose */
        if(last_frame_)
        {
            current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
        }

        // Track keypoints features of last frame into current frame
        int num_track_last = TrackLastFrame();
        
        // Estimate position of current frame in world coordinate
        tracking_inliers_ = EstimateCurrentPose();

        // Determine status of tracking
        if (tracking_inliers_ > num_features_tracking_)
        {
            // Tracking good
            status_ = FrontendStatus::TRACKING_GOOD;
        }
        else if (tracking_inliers_ > num_features_tracking_bad_) 
        {
            // Tracking bad
            status_ = FrontendStatus::TRACKING_BAD;
        } 
        else 
        {
            // Lost
            status_ = FrontendStatus::LOST;
        }

        // Check if current frame is keyframe
        InsertKeyframe();

        // Relative pose of current frame and last frame
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

        if (viewer_) 
        {
            viewer_->AddCurrentFrame(current_frame_);
        }

        return true;
    }

    bool Frontend::AddFrame(Frame::Ptr frame) 
    {
        // Update Frontend when a new frame comes

        // Update curremt frame
        current_frame_ = frame;

        switch (status_) 
        {
            case FrontendStatus::INITING:
                StereoInit();
                break;
            case FrontendStatus::TRACKING_GOOD:
            case FrontendStatus::TRACKING_BAD:
                Track();
                break;
            case FrontendStatus::LOST:
                Reset();
                break;
        }

        last_frame_ = current_frame_;
        return true;
    }

    bool Frontend::Reset() 
    {
        std::cout << "Reset is not implemented. " << std::endl;
        return true;
    }

    void Frontend::SetMap(Map::Ptr map)
    {
        map_ = map;
    }

    void Frontend::SetBackend(std::shared_ptr<Backend> backend)
    {
        backend_ = backend;
    }

    void Frontend::SetViewer(std::shared_ptr<Viewer> viewer)
    {
        viewer_ = viewer;
    }

    void Frontend::SetCameras(Camera::Ptr left, Camera::Ptr right) 
    {
        camera_left_ = left;
        camera_right_ = right;
    }

    FrontendStatus Frontend::GetStatus() const
    {
        return status_;
    }

}