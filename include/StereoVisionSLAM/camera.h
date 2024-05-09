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

            // camera intrinsic parameters
            double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};

            // baseline of stereo system
            double baseline_{0.0};

            // extrinsic parameters: Transformations from coordinate of 
            // reference camera of stereosystem to this camera coordinate system
            Sophus::SE3d pose_;

            // Transformations from this camera coordinate system
            // to coordinate of reference camera of the stereo vision system
            Sophus::SE3d pose_inv_;

            
            Camera();

            Camera(double fx, double fy, double cx, double cy,
                   double baseline, const Sophus::SE3d &pose);

            Sophus::SE3d pose() const;

            Eigen::Matrix3d K() const;

            friend std::ostream& operator<<(std::ostream &os, const Camera &obj);
    };


}


#endif