#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

//#include "StereoVisionSLAM/backend.h"
#include "StereoVisionSLAM/common_include.h"
#include "StereoVisionSLAM/dataset.h"
#include "StereoVisionSLAM/frontend.h"
#include "StereoVisionSLAM/viewer.h"
#include "StereoVisionSLAM/map.h"

namespace slam 
{

    class VisualOdometry
    {
        /* Stereo Visual SLAM pipeline */

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<VisualOdometry> Ptr;

            // Constructor with config file
            VisualOdometry(std::string &config_file_path);
            
            // Initialize different componets of stereo vision slam
            bool initialize();

            // Update stereo visual slam with new frame 
            bool step();

            // Process the whole input image sequence
            void run();

            FrontendStatus GetFrontendStatus() const;

        private:
            bool inited_{false};
            std::string config_file_path_;
            Frontend::Ptr frontend_{nullptr};
            //std::shared_ptr<Backend> backend_{nullptr};
            Map::Ptr map_{nullptr};
            Viewer::Ptr viewer_{nullptr};
            Dataset::Ptr dataset_{nullptr};
    };

}  

#endif