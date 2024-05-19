#ifndef CAMERA_H
#define CAMERA_H

#include "StereoVisionSLAM/common_include.h"

namespace slam
{

    class Camera
    {
        // Pinhole camera model

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Camera> Ptr;

            // Camera intrinsic parameters
            double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};

            // Baseline of stereo system
            double baseline_{0.0};

            // Extrinsic parameters: Transformations from the coordinate system
            // of the reference camera in the stereosystem to this camera's coordinate system
            Sophus::SE3d pose_;

            // Transformations from this camera coordinate system
            // to coordinate of reference camera of the stereo vision system
            Sophus::SE3d pose_inv_;


            Camera();

            Camera(double fx, double fy, double cx, double cy,
                   double baseline, const Sophus::SE3d &pose);

            Sophus::SE3d pose() const;

            Eigen::Matrix3d K() const;

            // world coordinate (map) to the camera coordiante
            Eigen::Vector3d world2camera(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w);

            // camera coordinate to world coordinate (map)
            Eigen::Vector3d camera2world(const Eigen::Vector3d &p_c, const Sophus::SE3d &T_c_w );

            Eigen::Vector2d camera2pixel(const Eigen::Vector3d &p_c);

            Eigen::Vector3d pixel2camera(const Eigen::Vector2d &p_p, double depth = 1);

            Eigen::Vector2d world2pixel(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w);

            Eigen::Vector3d pixel2world(const Eigen::Vector2d &p_p, const Sophus::SE3d &T_c_w, double depth);

            friend std::ostream& operator<<(std::ostream &os, const Camera &obj);
    };


}


#endif