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
            /* A frame is keyframe if number of tracked points from last 
             * frame to current frame be less than a threshold */
            bool is_keyframe_{false};
            /* Tcw: Transformation from world coordinate
             * to stereo system coordinate at this frame */
            Sophus::SE3d pose_;
            // Mutex for accessing pose
            std::mutex pose_mutex_;
            // Left and right images in stereo system
            cv::Mat left_img_, right_img_;
            /* Left image extracted feature vector
             * representation by a neural network */
            Eigen::Matrix<float, 1280, 1> representation_vec_;
            bool has_rep_vec_{false};

            double time_stamp_;

            // Extracted features in left image
            std::vector<Feature::Ptr> feature_left_;
            // Extracted features in right image, set to nullptr if no corresponding
            std::vector<Feature::Ptr> feature_right_;

            Frame(){}
            Frame(long id, double time_stamp, const Sophus::SE3d &pose,
                  const cv::Mat &left_img, const cv::Mat &right_img);
            Frame& operator=(const Frame& other);
            
            Sophus::SE3d Pose();
            void SetPose(const Sophus::SE3d &pose);

            // Factory construction pattern, assigning IDs
            static Ptr CreateFrame();
            
            void SetKeyFrame();

    };

}

#endif