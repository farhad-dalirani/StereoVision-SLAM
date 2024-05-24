#ifndef BACKEND_H
#define BACKEND_H

#include "StereoVisionSLAM/common_include.h"
#include "StereoVisionSLAM/frame.h"
#include "StereoVisionSLAM/map.h"
#include "StereoVisionSLAM/camera.h"

namespace slam
{

    class Map;

    class Backend
    {
        /** 
         * Backend has a separate optimization thread, which starts
         * optimization when the Map is updated. Map updates are triggered
         * by the frontend
         */
        
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Backend> Ptr;

            Backend();

        private:
            Map::Ptr map_;
            std::thread backend_thread_;
            std::mutex data_mutex_;
            
            std::condition_variable map_update;
            std::atomic<bool> backend_running_;
            
            Camera::Ptr cam_left_{nullptr};
            Camera::Ptr cam_right_{nullptr};
    };

}

#endif