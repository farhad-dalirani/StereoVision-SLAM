#include<StereoVisionSLAM/camera.h>

namespace slam 
{

    Camera::Camera(double fx, double fy, double cx, double cy,
            double baseline, const Sophus::SE3d &pose)
            : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose)
            {
                 pose_inv_ = pose_.inverse();
            }

    Eigen::Matrix3d Camera::K() const
    {
        // return Camera instrinsic matrix
        
        Eigen::Matrix3d k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    Sophus::SE3d Camera::pose() const
    {
        return pose_;
    }

    std::ostream& operator<<(std::ostream &os, const Camera &obj)
    {   
        os << "fx: " << obj.fx_ <<
              ", fy: " << obj.fy_ <<
              ", cx: " << obj.cx_ <<
              ", cy: " << obj.cy_ <<
              ", baseline: " << obj.baseline_ <<
              "\npose-r:\n" << obj.pose_.rotationMatrix() <<
              "\npose-t:\n" << obj.pose_.translation();
        return os;
    }
}