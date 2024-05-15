#include "gtest/gtest.h"
#include "StereoVisionSLAM/camera.h"
#include "StereoVisionSLAM/common_include.h"

using namespace slam;

// Test fixture for Camera class
class CameraTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        // Initialize test data
        fx = 500.0;
        fy = 500.0;
        cx = 320.0;
        cy = 240.0;
        // consider pose as identity
        baseline = 0.1;
        camera.reset(new Camera(fx, fy, cx, cy, baseline, pose));
    }

    // Test data
    double fx, fy, cx, cy, baseline;
    Sophus::SE3d pose;
    Camera::Ptr camera;
};

// Test the camera intrinsic matrix calculation
TEST_F(CameraTest, TestIntrinsicMatrix) {
    Eigen::Matrix3d K_expected;
    K_expected << fx, 0, cx,
                  0, fy, cy,
                  0, 0, 1;

    Eigen::Matrix3d K_actual = camera->K();

    ASSERT_EQ(K_expected, K_actual);
}

// Test the transformation from world to camera coordinates and vice versa
TEST_F(CameraTest, TestWorldToCameraVersa) {
    Eigen::Vector3d p_w(1.0, 2.0, 3.0);
    // Example rotation for testing
    Sophus::SE3d T_c_w = Sophus::SE3d::rotZ(M_PI / 4.0); 

    Eigen::Vector3d p_c_actual = camera->world2camera(p_w, T_c_w);

    // You may want to calculate the expected result manually or using another method
    // and then compare it with the actual result
    Eigen::Vector3d p_c_expected = pose * T_c_w * p_w;

    ASSERT_TRUE(p_c_actual.isApprox(p_c_expected, 1e-6));

    // Test camera to world coordinate
    Eigen::Vector3d p_w_estimated = camera->camera2world(p_c_actual, T_c_w);
    ASSERT_TRUE(p_w_estimated.isApprox(p_w, 1e-6));
}

TEST_F(CameraTest, TestCameraToPixelVersa) {
    Eigen::Vector3d p_c(20, 21, 22);

    Eigen::Vector3d p_x_1 = camera->K() * p_c;
    Eigen::Vector2d p_x (p_x_1(0,0)/p_x_1(2,0),p_x_1(1,0)/p_x_1(2,0));
    
    Eigen::Vector2d p_x_actual = camera->camera2pixel(p_c);

    ASSERT_TRUE(p_x_actual.isApprox(p_x, 1e-6));

    Eigen::Vector3d p_c_est = camera->pixel2camera(p_x_actual, p_c(2,0));

    ASSERT_TRUE(p_c_est.isApprox(p_c, 1e-6));
}