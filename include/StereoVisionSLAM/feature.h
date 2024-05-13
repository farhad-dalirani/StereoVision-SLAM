#ifndef FEATURE_H
#define FEATURE_H

#include "StereoVisionSLAM/common_include.h"
#include <opencv2/features2d.hpp>

namespace slam
{

    // Avoid circular dependency error
    class Frame;
    class MapPoint;

    class Feature 
    {
        // 2D feature in image for feature tracking/matching 

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Feature> Ptr;

            // Feature point parameters like 2d location
            cv::KeyPoint position_;
            // Pointer to the frame where the feature was found
            std::weak_ptr<Frame> frame_;
            // Pointer to corresponding point in constructed map
            std::weak_ptr<MapPoint> map_point_;

            bool outlier_{false};
            bool is_on_left_image_{true};

            Feature(){}
            Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp);

    };

}

#endif