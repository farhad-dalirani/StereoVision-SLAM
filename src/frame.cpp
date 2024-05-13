#include<StereoVisionSLAM/frame.h>

namespace slam
{
    Frame::Frame(long id, double time_stamp, const Sophus::SE3d &pose,
          const cv::Mat &left_img, const cv::Mat &right_img)
          :id_(id), time_stamp_(time_stamp), pose_(pose), 
          left_img_(left_img), right_img_(right_img){}

    Frame::Ptr Frame::CreateFrame()
    {
        static long factory_id{0};
        Ptr new_frame = std::make_shared<Frame>();
        new_frame->id_ = factory_id++;
        return new_frame;
    }

    Frame& Frame::operator=(const Frame& other) 
    {
        // Check for self-assignment
        if (this != &other) 
        {
            // Copy data members from 'other' to 'this'
            id_ = other.id_;
            keyframe_id_ = other.keyframe_id_;
            is_keyframe = other.is_keyframe;
            pose_ = other.pose_;
            // Deep copy of cv::Mat
            left_img_ = other.left_img_.clone(); 
            right_img_ = other.right_img_.clone(); 
            time_stamp_ = other.time_stamp_;
            // Deep copy of feature vectors
            feature_left_.clear();
            for (const auto& feature : other.feature_left_) 
            {
                feature_left_.push_back(std::make_shared<Feature>(*feature));
            }
            feature_right_.clear();
            for (const auto& feature : other.feature_right_) 
            {
                if (feature)
                    feature_right_.push_back(std::make_shared<Feature>(*feature));
                else
                    feature_right_.push_back(nullptr);
            }
        }
        return *this;
    }

}