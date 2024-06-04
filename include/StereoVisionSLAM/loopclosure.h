#ifndef LOOPCLOSURE_H
#define LOOPCLOSURE_H

#include<StereoVisionSLAM/common_include.h>
#include<StereoVisionSLAM/map.h>
#include<StereoVisionSLAM/camera.h>
#include<StereoVisionSLAM/viewer.h>

namespace slam
{
    
    class LoopClosure
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<LoopClosure> Ptr;

            // Initialize loop closure optimization in a seprate thread
            LoopClosure();
            // Signal for one loop closure optimization
            void UpdateMap();
            // Close loop closure optimization
            void Stop(); 

            void SetMap(std::shared_ptr<Map> map);
            void SetViewer(std::shared_ptr<Viewer> viewer);
            void SetCameras(Camera::Ptr left, Camera::Ptr right);

        private:
            void LoopClosureLoop();

            Map::Ptr map_{nullptr};
            std::shared_ptr<Viewer> viewer_{nullptr};

            std::thread loopclosure_thread_;
            std::mutex data_mutex_;
            
            std::condition_variable map_update_;
            std::atomic<bool> loopclosure_running_;
            
            Camera::Ptr cam_left_{nullptr};
            Camera::Ptr cam_right_{nullptr};
    };

}

#endif