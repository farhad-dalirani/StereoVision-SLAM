#ifndef BACKEND_H
#define BACKEND_H

#include "StereoVisionSLAM/common_include.h"
#include "StereoVisionSLAM/frame.h"
#include "StereoVisionSLAM/map.h"
#include "StereoVisionSLAM/viewer.h"
#include "StereoVisionSLAM/camera.h"

namespace slam
{

    class Map;

    class Backend
    {
        /** 
         * Backend has a separate optimization thread, which starts
         * optimization when the Map is updated. Map updates are triggered
         * by the frontend. It optimizes active keyframes and active map points.
         */
        
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Backend> Ptr;

            // Initialize backend optimization in a seprate thread
            Backend();
            // Signal for one backend optimization
            void UpdateMap();
            // Close backend optimization
            void Stop(); 

            void SetMap(std::shared_ptr<Map> map);
            void SetViewer(std::shared_ptr<Viewer> viewer);
            void SetCameras(Camera::Ptr left, Camera::Ptr right);


        private:
            Map::Ptr map_{nullptr};
            std::shared_ptr<Viewer> viewer_{nullptr};

            std::thread backend_thread_;
            std::mutex data_mutex_;
            
            std::condition_variable map_update_;
            std::atomic<bool> backend_running_;
            
            Camera::Ptr cam_left_{nullptr};
            Camera::Ptr cam_right_{nullptr};

            // Robust Kernel threshold
            double chi2_th_{5.991};

            // Refine pose of given keyframes and landmarks (bundle adjustment)
            void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks); 
            /* Waits for a signal to run the backend optimization on
             * active keyframes and active landmarks */
            void BackendLoop();
    };

}

#endif