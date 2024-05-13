#ifndef FRAME_H
#define FRAME_H

#include "StereoVisionSLAM/common_include.h"
#include "StereoVisionSLAM/feature.h"

namespace slam
{
    class Frame
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Frame> Ptr;

            // ID of frame
            unsigned long id_{0};
            // ID of key frame
            unsigned long keyframe_id_{0};
            // A frame is keyframe if tracked points be less 
            // than a threshold
            bool is_keyframe{false};
            // Tcw: Transformation from world to frame 
            Sophus::SE3d pose_;
            // mutex for accessing pose
            std::mutex pose_mutex_;
            // left and right images in stereo system
            cv::Mat left_img_, right_img_;
            double time_stamp_;

            // Extracted features in left image
            std::vector<std::shared_ptr<Feature>> feature_left_;
            // Extracted features in right image, set to nullptr if no corresponding
            std::vector<std::shared_ptr<Feature>> feature_right_;

            Frame(){}
            Frame(long id, double time_stamp, const Sophus::SE3d &pose,
                  const cv::Mat &left_img, const cv::Mat &right_img);
            Frame& operator=(const Frame& other);
            
            // Factory construction pattern, assigning IDs
            static Ptr CreateFrame();

    };

}

#endif