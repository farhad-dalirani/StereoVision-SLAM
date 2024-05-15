#include <gtest/gtest.h>
#include "StereoVisionSLAM/frame.h"

// Test Fixture for Frame class
class FrameTest : public ::testing::Test 
{
protected:
    virtual void SetUp() {
        // Create a sample frame for testing
        id = 1;
        time_stamp = 123.456;
        pose = Sophus::SE3d();
        left_img = cv::Mat(480, 640, CV_8UC1, cv::Scalar(127));
        right_img = cv::Mat(480, 640, CV_8UC1, cv::Scalar(255));
        frame = slam::Frame(id, time_stamp, pose, left_img, right_img);
    }

    // Variables for testing
    long id;
    double time_stamp;
    Sophus::SE3d pose;
    cv::Mat left_img, right_img;
    slam::Frame frame;
};

// Test constructor
TEST_F(FrameTest, Constructor) {
    EXPECT_EQ(frame.id_, id);
    EXPECT_EQ(frame.time_stamp_, time_stamp);
    // EXPECT_EQ(frame.pose_, pose);
    EXPECT_EQ(frame.left_img_.rows, left_img.rows);
    EXPECT_EQ(frame.left_img_.cols, left_img.cols);
    EXPECT_EQ(frame.left_img_.type(), left_img.type());
    EXPECT_EQ(frame.right_img_.rows, right_img.rows);
    EXPECT_EQ(frame.right_img_.cols, right_img.cols);
    EXPECT_EQ(frame.right_img_.type(), right_img.type());
}

// Test keyframe properties
TEST_F(FrameTest, KeyframeProperties) {
    EXPECT_FALSE(frame.is_keyframe_); // By default, should not be a keyframe
    frame.is_keyframe_ = true;
    EXPECT_TRUE(frame.is_keyframe_);
}

TEST_F(FrameTest, CreateFrameAndSetKeyFrame) {
    
    slam::Frame::Ptr a = slam::Frame::CreateFrame();
    EXPECT_EQ(a->id_, 0);
    
    slam::Frame::Ptr b = slam::Frame::CreateFrame();
    EXPECT_EQ(b->id_, 1);

    a->SetKeyFrame();
    EXPECT_EQ(a->keyframe_id_, 0);
    EXPECT_EQ(a->is_keyframe_, true);
    
    b->SetKeyFrame();
    EXPECT_EQ(b->keyframe_id_, 1);
    EXPECT_EQ(b->is_keyframe_, true);

}

TEST_F(FrameTest, SetAndGetPose) {
    
    // Create a rotation matrix representing a rotation of 90 degrees around the Z-axis
    Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Create a translation vector
    Eigen::Vector3d translation(1, 2, 3);

    // Create an SE3d object representing the transformation
    Sophus::SE3d se3(rotation_matrix, translation);

    slam::Frame::Ptr a = slam::Frame::CreateFrame();
    
    a->SetPose(se3);

    auto se3_returned = a->Pose();

    ASSERT_EQ((se3_returned.inverse() * se3).log().norm(), 0);
}