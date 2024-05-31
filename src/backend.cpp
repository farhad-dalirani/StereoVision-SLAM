#include "StereoVisionSLAM/backend.h"
#include "StereoVisionSLAM/g2o_types.h"
#include "StereoVisionSLAM/algorithm.h"

namespace slam
{

    void Backend::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks) 
    {
        /* 
         * In backend optimization (Boundle Adjustment), keyframes' pose and  
         * landmarks' position in world coordinate system are refined
         * through using non-linear least square optimization. It 
         * optimizes by minimizing the projection error of landmarks
         * and their corresponding tracked features. 
         */

       /* A 6*3 block is used because the optimized parameters represent the
         * current frame's  pose as a 6D vector (Lie algebra: 3 for translation
         * + 3 for rotation), and the observations are 3D landmarks (map points). */
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        
        // Linear solver for solving linear subproblem during no-linear optimization
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        /* Levenberg-Marquardt algorithm, which is used for nonlinear least squares optimization
         * Memory deallocation handled by optimizer */
        g2o::OptimizationAlgorithmLevenberg* solver = 
            new g2o::OptimizationAlgorithmLevenberg(
                std::make_unique<BlockSolverType>(
                    std::make_unique<LinearSolverType>()));

        // Optimizer
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // Add keyframe vertices to optimization graph
        std::map<unsigned long, VertexPose *> vertices;
        unsigned long max_kf_id{0};
        for(auto &keyframe: keyframes)
        {
            // Retreive keyframe
            Frame::Ptr kf = keyframe.second;

            // Create Pose vertex, add initial parameter with frame's pose
            VertexPose *vertex_pose = new VertexPose(); // Memory deallocation managed by g2o
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose());
            
            // Add new vertex to graph
            optimizer.addVertex(vertex_pose);

            if(kf->keyframe_id_ > max_kf_id)
            {
                max_kf_id = kf->keyframe_id_;
            }  

            vertices.insert({kf->keyframe_id_, vertex_pose});
        }

        // Camera intrinsic parameter
        Eigen::Matrix3d K_left = cam_left_->K();
        Eigen::Matrix3d K_right = cam_right_->K();

        // Camera pose in reference coordinate frame of stereo system
        Sophus::SE3d left_ext = cam_left_->pose();
        Sophus::SE3d right_ext = cam_right_->pose();

        // Add landmark vertices and edges to optimization graph
        std::map<unsigned long, VertexXYZ *> vertices_landmarks;
        int index{1};
        double chi2_th{5.991}; // Robust Kernel threshold
        std::map<EdgeProjection *, Feature::Ptr> edges_and_features;
        for(auto &landmark: landmarks)
        {
            // if landmark (3D map point) is outlier, discard it
            if(landmark.second->is_outlier_)
            {
                continue;
            }

            unsigned int landmark_id{landmark.second->id_};

            /* Obtain 2D keypoint features accross different frames
             * that are associated to the landmark */
            std::list<std::weak_ptr<Feature>> observations = landmark.second->GetObs();

            for(std::weak_ptr<Feature> &obs: observations)
            {
                Feature::Ptr feat = obs.lock();
                if(feat == nullptr)
                {
                    continue;
                }
                if(feat->outlier_ == true)
                {
                    continue;
                }

                // Frame that 2d keypoint feature is in it
                Frame::Ptr frame = feat->frame_.lock();
                if(frame == nullptr)
                {
                    continue;
                }

                /* If the landmark has not yet been added to 
                 * optimization graph, add a new vertex */
                if(vertices_landmarks.find(landmark_id) == vertices_landmarks.end())
                {
                    /* Create landmark vertex and initialize it
                     * with current landmark position */
                    VertexXYZ *v = new VertexXYZ();
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true); // Schur method, neccessary for utilizing sparse nature of optimization
                    
                    optimizer.addVertex(v);
                    
                    vertices_landmarks.insert({landmark_id, v});
                }   
                
                // Add edge to optimization graph
                if(vertices.find(frame->keyframe_id_) != vertices.end())
                {
                    EdgeProjection *edge = nullptr;
                    if(feat->is_on_left_image_)
                    {
                        edge = new EdgeProjection(K_left, left_ext);
                    }
                    else
                    {
                        edge = new EdgeProjection(K_right, right_ext);
                    }

                    edge->setId(index);
                    edge->setVertex(0, vertices.at(frame->keyframe_id_)); // pose
                    edge->setVertex(1, vertices_landmarks.at(landmark_id)); // landmark
                    edge->setMeasurement(toVec2(feat->position_.pt));
                    edge->setInformation(Eigen::Matrix2d::Identity());
                    auto rk = new g2o::RobustKernelHuber();
                    rk->setDelta(chi2_th);
                    edge->setRobustKernel(rk);
                    optimizer.addEdge(edge);
                    
                    edges_and_features.insert({edge, feat});
                    
                    index++;
                }
            }
        }

        // Do optimization
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        // Detect keypoint features that are outliers
        int cnt_outlier {0}, cnt_inlier {0};
        int iteration {0};
        while (iteration < 5) 
        {
            cnt_outlier = 0;
            cnt_inlier = 0;
            // determine if we want to adjust the outlier threshold
            for (auto &ef : edges_and_features) 
            {
                if (ef.first->chi2() > chi2_th) 
                {
                    cnt_outlier++;
                } else {
                    cnt_inlier++;
                }
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.5) 
            {
                break;
            } 
            else 
            {
                chi2_th *= 2;
                iteration++;
            }
        }

        /* Remove link between keypoint features that are outlier
         * and their corresponding map point */
        for (auto &ef : edges_and_features) 
        {
            if (ef.first->chi2() > chi2_th) 
            {
                ef.second->outlier_ = true;
                // Remove the observation-map point link
                MapPoint::Ptr map_p = ef.second->map_point_.lock();
                if(map_p)
                {
                    map_p->RemoveObservation(ef.second);
                }
            } 
            else 
            {
                ef.second->outlier_ = false;
            }
        }

        if(viewer_)
        {
            viewer_->LogInfoMKF("Backend: Outlier/Inlier in optimization " + 
                            std::to_string(cnt_outlier) + "/" + std::to_string(cnt_inlier), max_kf_id);
        }

        // Update keyframes' pose and landmarks' position with optimized vaules 
        for (auto &v : vertices) 
        {
            keyframes.at(v.first)->SetPose(v.second->estimate());
        }
        for (auto &v : vertices_landmarks) 
        {
            landmarks.at(v.first)->SetPos(v.second->estimate());
        }

    }

    void Backend::BackendLoop()
    {
        /* Continously waits for a signal to run the backend
         * optimization as requested. It only apply bounle adjustment optimization
         * on active key frames and active landmarks */
        while(backend_running_.load())
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            
            // Wait untill a backend optimization requested
            map_update_.wait(lock);

            /* optimized only active keyframes and active landmarks */
            Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
            Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
            Optimize(active_kfs, active_landmarks);
        }
        
    }

    void Backend::UpdateMap() 
    {
        // Signal for one backend optimization
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.notify_one();
    }

    void Backend::Stop() 
    {
        // Close backend optimization
        backend_running_.store(false);
        map_update_.notify_one();
        backend_thread_.join();
    }

    Backend::Backend()
    {
        // Initialize backend optimization in a seprate thread
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    }

    void Backend::SetCameras(Camera::Ptr left, Camera::Ptr right) 
    {
        cam_left_ = left;
        cam_right_ = right;
    }

    void Backend::SetMap(Map::Ptr map) 
    { 
        map_ = map; 
    }

    void Backend::SetViewer(std::shared_ptr<Viewer> viewer)
    {
        viewer_ = viewer;
    }

}