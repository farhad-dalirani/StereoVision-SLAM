#include<StereoVisionSLAM/feature.h>

namespace slam
{
    Feature::Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
            :frame_(frame), position_(kp){}
}