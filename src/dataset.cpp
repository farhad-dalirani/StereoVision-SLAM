#include<StereoVisionSLAM/dataset.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "StereoVisionSLAM/slamexception.h"
#include "StereoVisionSLAM/config.h"

namespace slam
{

    Dataset::Dataset(const std::string &dataset_path)
    : dataset_path_(dataset_path)
    {
        // Set index of left and right cameras among cameras in stereo vision system
        left_cam_index_ = Config::Get<int>("left_cam_index");
        right_cam_index_ = Config::Get<int>("right_cam_index");

        if(Config::Get<int>("is_color_input") != 0)
        {
            flag_read_img_ = cv::IMREAD_COLOR;
        }
    }

    bool Dataset::initialize()
    {
        // Read camera intrinsic and extrinsic parameters

        /* Open file that contain 3*4 projection matrix
         * of each camera in KITTI dataset */
        std::ifstream fin(dataset_path_ + "/calib.txt");
        if(!fin)
        {
            throw SLAMException("Cannot open KITTI camera parameters file (calib.txt).");
        }

        /* There are four cameras in the KITTI dataset stereo vision system.
         * 0, 1: left and right monocular cameras
         * 2, 3: left and right color cameras */
        for(int i{0}; i<4; ++i)
        {
            char cam_name[3];
            fin >> cam_name[0] >> cam_name[1] >> cam_name[2];
            cam_name[2] = '\0';

            // Read projection matix of i'th camera 
            double pr_mat[12];
            for(int j{0}; j<12; ++j)
            {
                fin >> pr_mat[j];
            }

            /* Since images are rectified in the KITTI dataset,
             * the projection matrix general form from K[R|t] simplifies
             * to K[I|t] or [K|Kt]. */
            
            // Read camera instrinsic matrix
            Eigen::Matrix3d K;
            K << pr_mat[0], pr_mat[1], pr_mat[2], 
                 pr_mat[4], pr_mat[5], pr_mat[6], 
                 pr_mat[8], pr_mat[9], pr_mat[10];

            // Obtrain translation extrinsic parameter
            Eigen::Vector3d t;
            t << pr_mat[3], pr_mat[7], pr_mat[11];
            t = K.inverse() * t;

            /* baseline between i'th camera and reference camera
             * in stereo vision system */
            double baseline{t.norm()};

            /* Adopt K since we use down-sampled version of KITTI images
             * with factor of two */
            K *= 0.5;

            Camera::Ptr new_camera = std::make_shared<Camera>(
                K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                baseline, Sophus::SE3d(Sophus::SO3d(), t));
            
            cameras_.push_back(new_camera);
        }

        fin.close();
        current_image_index_ = 0;
        
        return true;
    }

    Camera::Ptr Dataset::GetCamera(int camera_id) const
    {
        return cameras_.at(camera_id);
    }

    std::string Dataset::GetDataDir()
    {
        return dataset_path_;
    }
    
    int Dataset::GetLeftCamIndex()
    {
        return left_cam_index_;
    }


    Frame::Ptr Dataset::NextFrame()
    {
        // Create and return next frame in video sequence

        std::ostringstream file_name_1;
        std::ostringstream file_name_2;

        // Path of left and right camera images
        file_name_1 << dataset_path_ << "/image_" << left_cam_index_ << 
                 "/" << std::setw(6) << std::setfill('0') << current_image_index_ << ".png";
        file_name_2 << dataset_path_ << "/image_" << right_cam_index_ << 
                "/" << std::setw(6) << std::setfill('0') << current_image_index_ << ".png";

        // Load images, assumed they are already rectified and undistorted.
        cv::Mat left_img = cv::imread(file_name_1.str(), flag_read_img_);
        cv::Mat right_img = cv::imread(file_name_2.str(), flag_read_img_);

        if ((left_img.data == nullptr) or (right_img.data == nullptr))
        {
            return nullptr;
        }

        // Down-sample images by factor of two for computational efficiency
        cv::Mat left_img_resized, right_img_resized;
        cv::resize(left_img, left_img_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
        cv::resize(right_img, right_img_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

        // Create new frame
        Frame::Ptr new_frame = Frame::CreateFrame();
        new_frame->left_img_ = left_img_resized;
        new_frame->right_img_ = right_img_resized;
        current_image_index_++;

        return new_frame;
    }

    Frame::Ptr Dataset::FrameById(unsigned long frame_id)
    {
        // Create and return i-th frame in video sequence

        std::ostringstream file_name_1;
        std::ostringstream file_name_2;

        // Path of left and right camera images
        file_name_1 << dataset_path_ << "/image_" << left_cam_index_ << 
                 "/" << std::setw(6) << std::setfill('0') << frame_id << ".png";
        file_name_2 << dataset_path_ << "/image_" << right_cam_index_ << 
                "/" << std::setw(6) << std::setfill('0') << frame_id << ".png";

        // Load images, assumed they are already rectified and undistorted.
        cv::Mat left_img = cv::imread(file_name_1.str(), flag_read_img_);
        cv::Mat right_img = cv::imread(file_name_2.str(), flag_read_img_);

        if ((left_img.data == nullptr) or (right_img.data == nullptr))
        {
            return nullptr;
        }

        // Down-sample images by factor of two for computational efficiency
        cv::Mat left_img_resized, right_img_resized;
        cv::resize(left_img, left_img_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
        cv::resize(right_img, right_img_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

        // Create new frame
        Frame::Ptr new_frame = Frame::CreateFrame();
        new_frame->left_img_ = left_img_resized;
        new_frame->right_img_ = right_img_resized;

        return new_frame;
    }

}
