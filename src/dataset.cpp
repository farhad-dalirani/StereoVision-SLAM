#include<StereoVisionSLAM/dataset.h>
#include <fstream>

namespace slam
{

    Dataset::Dataset(const std::string &dataset_path)
    : dataset_path_(dataset_path)
    {
    }

    void Dataset::initialize()
    {
        // Read camera intrinsic and extrinsic parameters

        // Open file that contain 3*4 projection matrix
        // of each camera in KITTI dataset
        std::ifstream fin(dataset_path_ + "/calib.txt");
        if(!fin)
        {
            throw SLAMException("Cannot open KITTI camera parameters file (calib.txt).");
        }

        // There are four cameras in the KITTI dataset stereo vision system.
        // 0, 1: left and right monocular cameras
        // 2, 3: left and right color cameras
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

            // Since images are rectified in the KITTI dataset,
            // the projection matrix general form from K[R|t] simplifies
            // to K[I|t] or [K|Kt].
            
            // Read camera instrinsic matrix
            Eigen::Matrix3d K;
            K << pr_mat[0], pr_mat[1], pr_mat[2], 
                 pr_mat[4], pr_mat[5], pr_mat[6], 
                 pr_mat[8], pr_mat[9], pr_mat[10];

            // Obtrain translation extrinsic parameter
            Eigen::Vector3d t;
            t << pr_mat[3], pr_mat[7], pr_mat[11];
            t = K.inverse() * t;

            // baseline between i'th camera and reference camera
            // in stereo vision system
            double baseline{t.norm()};

            // Adopt K since we use down-sampled version of KITTI images
            // with factor of two
            K *= 0.5;

            Camera::Ptr new_camera = std::make_shared<Camera>(
                K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                baseline, Sophus::SE3d(Sophus::SO3d(), t));
            std::cout << *new_camera << std::endl;
            cameras_.push_back(new_camera);
        }

        fin.close();
        current_image_index_ = 0;
    }

}
