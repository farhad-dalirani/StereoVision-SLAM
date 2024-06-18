#ifndef DENSE_RECONSTRUCTION_H
#define DENSE_RECONSTRUCTION_H

#include "StereoVisionSLAM/common_include.h"
#include "StereoVisionSLAM/dataset.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace slam
{
    class DenseReconstruction
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<DenseReconstruction> Ptr;
            typedef pcl::PointXYZRGB PointT;
            typedef pcl::PointCloud<PointT> PointCloud;

            DenseReconstruction(std::string config_file_path);
            void Initialize(); 
            void DenseReconstruct();

        private:
            // Path to config file
            std::string config_file_path_;
            /* Path to a folder that contains output 
            * of the SLAM pipeline */
            std::string slam_output_dir_;
            /* Index of left and right cameras among stereo system cameras
             * during dense 3D reconstruction */
            int left_cam_index_;
            int right_cam_index_;
            /* Data squence path that SLAM executed on it and is input
             * to 3D dense reconstruction (In output of SLAM pipeline) */
            std::string data_sequence_path_;
            /* The index of left camera among cameras of stereo system
             * that was used during SLAM  (In output of SLAM pipeline) */
            int left_camera_index_in_slam_;
            // Dataset object
            Dataset::Ptr dataset_{nullptr};
            // Poses of keyframes (output of SLAM pipeline)
            std::vector<Sophus::SE3f> keyframes_poses_; 
            // Image id of each keyframes' images (In output of SLAM pipeline)
            std::vector<unsigned long> keyframes_images_id_;

            // Create a point cloud
            PointCloud::Ptr pointcloud_;

            // Stereo depth estimator 
            int num_disparities_{128};
            int blockSize_{15};
            cv::Ptr<cv::StereoBM> stereo_depth_est_;
    };

}

#endif