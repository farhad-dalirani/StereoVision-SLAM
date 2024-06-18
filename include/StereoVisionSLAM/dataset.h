#ifndef DATASET_H
#define DATASET_H

#include "StereoVisionSLAM/common_include.h"
#include "StereoVisionSLAM/camera.h"
#include "StereoVisionSLAM/frame.h"

namespace slam
{

    class Dataset
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Dataset> Ptr;

            Dataset(const std::string &dataset_path);

            // Read dataset information including cameras parameters
            bool initialize();

            // Create and return next frame in sequence
            Frame::Ptr NextFrame();
            
            // Create and return i-th frame in sequence
            Frame::Ptr FrameById(unsigned long frame_id);

            // Get camera by id
            Camera::Ptr GetCamera(int camera_id) const;

            std::string GetDataDir();
            int GetLeftCamIndex();

        private:
            // Root path of video sequence in KITTI
            std::string dataset_path_;
            // KITTI has 4 cameras, we used cameras 0 and 1 by default
            int left_cam_index_{0};
            int right_cam_index_{1};
            // Flag for reading gray/color images
            int flag_read_img_{cv::IMREAD_GRAYSCALE};
            // Parameters of 4 cameras of KITTI
            std::vector<Camera::Ptr> cameras_;
            // Indicates the index of the image to be read from the KITTI sequence 
            int current_image_index_{0};
            
    };

}

#endif