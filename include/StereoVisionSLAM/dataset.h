#ifndef DATASET_H
#define DATASET_H

#include "StereoVisionSLAM/common_include.h"
#include "StereoVisionSLAM/camera.h"

namespace slam
{

    class Dataset
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Dataset> Ptr;

            Dataset(const std::string &dataset_path);

            void initialize();

        private:
            std::string dataset_path_;
            int current_image_index_{0};
            std::vector<Camera::Ptr> cameras_;
    };

}

#endif