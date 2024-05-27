#ifndef VIEWER_H
#define VIEWER_H

#include "StereoVisionSLAM/common_include.h"
#include "StereoVisionSLAM/map.h"
#include "StereoVisionSLAM/camera.h"
#include "StereoVisionSLAM/frame.h"

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>


namespace slam
{
    class Viewer 
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Viewer> Ptr;

            Viewer();

            void SetMap(Map::Ptr map);

            void SetCameras(Camera::Ptr cameraLeft, Camera::Ptr cameraRight);

            void Close();

            void AddCurrentFrame(Frame::Ptr current_frame);

            void UpdateMap();

        private:
            
            const rerun::RecordingStream rec = rerun::RecordingStream("rerun_Stereo_Vision_SLAM");

            Frame::Ptr current_frame_ = nullptr;
            Map::Ptr map_ = nullptr;
            Camera::Ptr camera_left_ = nullptr;
            Camera::Ptr camera_right_ = nullptr;

            bool viewer_running_ = true;

            std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
            std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;

            std::mutex viewer_data_mutex_;
    };

}



#endif