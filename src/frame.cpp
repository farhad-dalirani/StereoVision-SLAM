#include<StereoVisionSLAM/frame.h>

namespace slam
{
    Frame::Frame(long id, double time_stamp, const Sophus::SE3d &pose,
          const cv::Mat &left_img, const cv::Mat &right_img)
          :id_(id), time_stamp_(time_stamp), pose_(pose), 
          left_img_(left_img), right_img_(right_img){}

    Sophus::SE3d Frame::Pose()
    {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void Frame::SetPose(const Sophus::SE3d &pose)
    {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    Frame::Ptr Frame::CreateFrame()
    {
        static long factory_id{0};
        Ptr new_frame = std::make_shared<Frame>();
        new_frame->id_ = factory_id++;
        return new_frame;
    }

    void Frame::SetKeyFrame()
    {
        static unsigned long keyframe_factory_id{0};
        is_keyframe_ = true;
        keyframe_id_ = keyframe_factory_id++;
    }

}