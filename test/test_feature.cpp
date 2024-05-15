#include "StereoVisionSLAM/frame.h"
#include "StereoVisionSLAM/feature.h"

#include <gtest/gtest.h>


// Mock class for Frame
class FrameMock : public slam::Frame {
public:
    FrameMock(long id, double time_stamp, const Sophus::SE3d &pose,
          const cv::Mat &left_img, const cv::Mat &right_img) :
          slam::Frame(id, time_stamp, pose, left_img, right_img) {}
};

// Test Fixture for Feature class
class FeatureTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        // Create a sample frame for testing
        frame = std::make_shared<FrameMock>(1, 123.456, Sophus::SE3d(), cv::Mat(), cv::Mat());
        // Create a sample keypoint
        cv::KeyPoint keypoint(100, 200, 2.5);
        feature = std::make_shared<slam::Feature>(frame, keypoint);
    }

    // Variables for testing
    std::shared_ptr<FrameMock> frame;
    std::shared_ptr<slam::Feature> feature;
};

// Test constructor
TEST_F(FeatureTest, Constructor) {
    ASSERT_EQ(feature->position_.pt.x, 100);
    ASSERT_EQ(feature->position_.pt.y, 200);
    ASSERT_EQ(feature->position_.size, 2.5);
    ASSERT_EQ(feature->frame_.lock(), frame);
    ASSERT_EQ(feature->map_point_.lock(), nullptr);
    ASSERT_FALSE(feature->outlier_);
    ASSERT_TRUE(feature->is_on_left_image_);
}

