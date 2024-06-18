#include "StereoVisionSLAM/dense_reconstruction.h"
#include "StereoVisionSLAM/frame.h"
#include "StereoVisionSLAM/config.h"
#include "StereoVisionSLAM/slamexception.h"
#include <fstream> 
#include <sstream> 

namespace slam
{
    DenseReconstruction::DenseReconstruction(std::string config_file_path)
    : config_file_path_(config_file_path)
    {
    }

    void DenseReconstruction::Initialize()
    {
        // Initialize attributes and component for 3D dense reconstruction class

        // Open config file for dense 3d reconstruction
        Config::SetParameterFile(config_file_path_);

        // Read path of SLAM pipeline's output folder
        slam_output_dir_ = Config::Get<std::string>("slam_output_dir");

        /* Index of left and right camera among all cameras that 
         * used for dense 3D reconstruction */
        left_cam_index_ = Config::Get<int>("left_cam_index");
        right_cam_index_ = Config::Get<int>("right_cam_index");


        // open SLAM pipeline's output
        std::ifstream slam_fp(slam_output_dir_);
        if(not(slam_fp))
        {
            throw SLAMException("Can not open SLAM pipeline's output file.");
        }
        // Read data sequence path
        slam_fp >> data_sequence_path_;
        // Index of left camera among cameras during SLAM
        slam_fp >> left_camera_index_in_slam_;

        // Read keyframes's poses and their images' id from file
        while(not(slam_fp.eof()))
        {            
            // Id of images in i-th keyframe
            unsigned long image_id;
            slam_fp >> image_id;
            if (slam_fp.fail()) 
            {
                break;
            }
            keyframes_images_id_.push_back(image_id);
           
            /* Read transformation matrix from world coordinate to 
             * i-th keyframe coodinate */
            Eigen::Matrix4f T = Eigen::Matrix4f::Zero();
            T(3, 3) = 1;
            for(int i{0}; i < 3; ++i)
            {
                for(int j{0}; j < 4; ++j)
                {
                    slam_fp >> T(i, j);
                    if (slam_fp.fail()) 
                    {
                        break;
                    }
                }
            }
            keyframes_poses_.push_back(Sophus::SE3f(T));
        }
        slam_fp.close();
        
        // Create Dataset object
        dataset_ = std::make_shared<Dataset>(Dataset(data_sequence_path_)); 
        
        // Initialize dataset
        if(!dataset_->initialize())
        {
            throw SLAMException("Cannot initialize object to read dataset.");
        }

        // Initialize map pointcloud
        pointcloud_ = std::make_shared<PointCloud>();
        
        // Initialize stereo depth estimator 
        stereo_depth_est_ = cv::StereoBM::create(num_disparities_, blockSize_);
    }

    void DenseReconstruction::DenseReconstruct()
    {
        /* */

        // For each keyframe
        for(int i{0}; i < keyframes_images_id_.size(); i++)
        {
            // Left and right color images
            Frame::Ptr keyframe_i = dataset_->FrameById(keyframes_images_id_[i]);

            /* compute disparity map for left and right images,
             * assumed they are already rectified and undistorted. */ 
            cv::Mat disparity_map;
            cv::Mat left_gray_img;
            cv::cvtColor(keyframe_i->left_img_, left_gray_img, cv::COLOR_BGR2GRAY);
            cv::Mat right_gray_img;
            cv::cvtColor(keyframe_i->right_img_, right_gray_img, cv::COLOR_BGR2GRAY);
            stereo_depth_est_->compute(left_gray_img, right_gray_img, disparity_map);
            // Converting disparity values to CV_32F from CV_16S
            disparity_map.convertTo(disparity_map, CV_32F, 1.0 / 16.0f);

            
            // Focal lenght
            float focal_length = dataset_->GetCamera(left_cam_index_)->fx_;
            // Baseline of left and right camera
            Eigen::Vector3d p1 = dataset_->GetCamera(left_cam_index_)->pose_inv_.translation();
            Eigen::Vector3d p2 = dataset_->GetCamera(right_cam_index_)->pose_inv_.translation();
            float baseline = (p1 - p2).norm();
            
            // Convert disparity to depth
            cv::Mat depthMap = cv::Mat::zeros(disparity_map.size(), CV_32F);
            for (int y = 0; y < disparity_map.rows; y++) 
            {
                for (int x = 0; x < disparity_map.cols; x++) 
                {
                    float disparityValue = disparity_map.at<float>(y, x);
                    if (disparityValue > 0) 
                    {
                        depthMap.at<float>(y, x) = (focal_length * baseline) / disparityValue;
                    } else 
                    {
                        // Handle invalid disparity values
                        depthMap.at<float>(y, x) = 0; 
                    }
                }
            }

            /* Convert pixels in left camera to points in stereo system 
             * coordinate system then to map coodinate */
            PointCloud::Ptr current = std::make_shared<PointCloud>();
            for(int x_i{0}; x_i < keyframe_i->left_img_.cols; x_i++)
            {
                for(int y_i{0}; y_i < keyframe_i->left_img_.rows; y_i++)
                {
                    if(depthMap.at<float>(y_i, x_i) < 1)
                    {
                        continue;
                    } 

                    Eigen::Vector2d pixel_xy(x_i, y_i);
                    Eigen::Vector3d map_point_i = dataset_->GetCamera(
                                                    left_cam_index_)->pixel2world(
                                                        pixel_xy, 
                                                        keyframes_poses_[i].cast<double>(),
                                                        static_cast<double>(depthMap.at<float>(y_i, x_i)));
                
                    // Add point to the this keyframe pointcloud
                    PointT p;
                    p.x = map_point_i[0];
                    p.y = map_point_i[1];
                    p.z = map_point_i[2];
                    p.b = keyframe_i->left_img_.data[y_i * keyframe_i->left_img_.step + x_i * keyframe_i->left_img_.channels()];
                    p.g = keyframe_i->left_img_.data[y_i * keyframe_i->left_img_.step + x_i * keyframe_i->left_img_.channels() + 1];
                    p.r = keyframe_i->left_img_.data[y_i * keyframe_i->left_img_.step + x_i * keyframe_i->left_img_.channels() + 2];
                    current->points.push_back(p);
                }
            }

            // Filter and statistical removal
            // it considers a neighbourhood around point
            // calculate mean and std of neighbourhood
            // removes the point if outside the distribution
            PointCloud::Ptr tmp(new PointCloud);
            pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
            statistical_filter.setMeanK(50);
            statistical_filter.setStddevMulThresh(1.0);
            statistical_filter.setInputCloud(current);
            statistical_filter.filter(*tmp);

            // Merge point could of current keyframe to map
            *pointcloud_ += *tmp;
        }

        // Save Dense 3D Reconstructed map 
        std::cout << "Saved " << pointcloud_->points.size() << " data points to test_pcd.pcd." << std::endl;
        pcl::io::savePCDFileBinary("test_pcd.pcd", *pointcloud_);
        
    }

}