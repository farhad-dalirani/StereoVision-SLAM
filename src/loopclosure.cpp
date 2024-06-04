
#include<StereoVisionSLAM/loopclosure.h>

namespace slam
{

    LoopClosure::LoopClosure()
    {
        // Set hyperparameters
        

        // Initialize backend optimization in a seprate thread
        loopclosure_running_.store(true);
        loopclosure_thread_ = std::thread(std::bind(&LoopClosure::LoopClosureLoop, this));
    }

    void LoopClosure::UpdateMap() 
    {
        // Signal for one loop closure optimization
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.notify_one();
    }

    void LoopClosure::LoopClosureLoop()
    {
        /* Continously waits for a signal to run the loop closure
         * optimization as requested. It only apply path optimization
         * on active key frames and active landmarks */
        while(loopclosure_running_.load())
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            
            // Wait untill a backend optimization requested
            map_update_.wait(lock);

            /* optimized only active keyframes and active landmarks */
            //Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
            //Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
            //Optimize(active_kfs, active_landmarks);

            
        }
    }

    void LoopClosure::Stop() 
    {
        // Close loop closure optimization
        loopclosure_running_.store(false);
        map_update_.notify_one();
        loopclosure_thread_.join();
    }

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

}