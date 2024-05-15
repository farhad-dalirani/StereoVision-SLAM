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
        // return pose of the camera in stereosystem.
        return pose_;
    }

    Eigen::Vector3d Camera::world2camera(
        const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w)
    {
        // Transform the point `p_w` from the world coordinate system (map)
        // to the camera coordinate system. `T_c_w` represents the pose of
        // the stero system in the world coordinate system. 
        // Here, `pose_` represents the pose of the camera in
        // stereosystem.
        return pose_ * T_c_w * p_w;
    }

    Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d &p_c, const Sophus::SE3d &T_c_w )
    {
        // Transform the point `p_c` from the camera coordinate to 
        // world coordinate system (map coordinate)
        return T_c_w.inverse() * pose_inv_ * p_c;
    }

    Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d &p_c)
    {
        // Calculate corresponding pixel of a point in camera coordinate
        // system

        // px = K * p_c
        return Eigen::Vector2d(fx_ * p_c(0, 0) / p_c(2, 0) + cx_, 
                               fy_ * p_c(1, 0) / p_c(2, 0) + cy_);
    }


    
    Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d &p_p, double depth)
    {
        // Find the corresponding 3D point of
        // a pixel in the camera coordinate system.
        // If depth is equal to 1, it finds the corresponding 
        // point of the pixel in the camera's normal plane.

        // p_camera_normal_plane = inverse(K) * p_p
        // p_c = p_camera_normal_plane * depth
        return Eigen::Vector3d(
            (p_p(0, 0) - cx_) * depth / fx_,
            (p_p(1, 0) - cy_) * depth / fy_,
            depth);
    }

    Eigen::Vector2d Camera::world2pixel(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w)
    {
        // Find correspondance of `p_w` from the world coordinate system (map)
        // in camera image. `T_c_w` represents the pose of
        // the stero system in the world coordinate system. 
        return camera2pixel(world2camera(p_w, T_c_w));
    }

    Eigen::Vector3d Camera::pixel2world(const Eigen::Vector2d &p_p, const Sophus::SE3d &T_c_w, double depth)
    {
        // Find correspondance of a pixel in world camera coordinate (map coordinate)
        return camera2world(pixel2camera(p_p, depth), T_c_w);
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