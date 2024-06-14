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

            // Log in rerun viewer
            void LogInfo(std::string msg, std::string log_type);

            // Log in rerun viewer with id of most recent active keyframe as index
            void LogInfoMKF(std::string msg, unsigned long maxkeyframe_id,
                             std::string log_type);

            void Plot(std::string plot_name, double value, unsigned long maxkeyframe_id);

        private:
            
            const rerun::RecordingStream rec = rerun::RecordingStream("rerun_Stereo_Vision_SLAM");

            Frame::Ptr current_frame_{nullptr};
            Map::Ptr map_{nullptr};
            Camera::Ptr camera_left_{nullptr};
            Camera::Ptr camera_right_{nullptr};

            bool viewer_running_{true};
            
            std::unordered_map<unsigned long, Frame::Ptr> all_keyframes_;
            std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
            std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;

            std::mutex viewer_data_mutex_;

            // Different color for loging info of different components
            std::unordered_map<std::string, rerun::Color> log_color{
                {"vo", rerun::Color(255, 255, 255)}, 
                {"frontend", rerun::Color(0, 255, 255)},
                {"backend", rerun::Color(0, 255, 0)},
                {"loopclosure", rerun::Color(255, 165, 0)}};
    };

}



#endif