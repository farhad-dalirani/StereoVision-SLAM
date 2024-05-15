#include <gtest/gtest.h> // Include Google Test framework
#include <StereoVisionSLAM/dataset.h>

// Define a test fixture
class DatasetTest : public ::testing::Test 
{
    protected:
        // Set up common objects or state for tests
        void SetUp() override {
            // Create Dataset object
            dataset = std::make_shared<slam::Dataset>("./data/dataset/sequences/00");
            // Initialize the dataset
            dataset->initialize(); 
        }

        // Clean up after the test
        void TearDown() override {
            // Any necessary cleanup code
        }

        // Declare objects that used during tests
        slam::Dataset::Ptr dataset;
};

// Test case to ensure Dataset initialization
TEST_F(DatasetTest, Initialization) 
{
    // Check if Dataset object is not null
    ASSERT_NE(dataset, nullptr);
    ASSERT_NE(dataset->GetCamera(3)->fx_, 0.0);
    ASSERT_NE(dataset->GetCamera(3)->fy_, 0.0);
    ASSERT_NE(dataset->GetCamera(3)->cx_, 0.0);
    ASSERT_NE(dataset->GetCamera(3)->cy_, 0.0);
    ASSERT_NE(dataset->GetCamera(3)->baseline_, 0.0);
    ASSERT_NE(dataset->GetCamera(0)->baseline_, dataset->GetCamera(1)->baseline_);
}

// Test case to ensure correct frame loading
TEST_F(DatasetTest, NextFrame) 
{
    // Load a frame from the dataset
    slam::Frame::Ptr frame1 = dataset->NextFrame();
    // Check if the loaded frame is not null and id_ is correct
    ASSERT_NE(frame1, nullptr);
    ASSERT_EQ(frame1->id_, 0);
    

    // Load a frame from the dataset
    slam::Frame::Ptr frame2 = dataset->NextFrame();
    // Check if the loaded frame is not null and id_ is correct
    ASSERT_NE(frame2, nullptr);
    ASSERT_EQ(frame2->id_, 1);
    
    ASSERT_EQ(frame2->left_img_.rows, frame2->right_img_.rows);
    ASSERT_EQ(frame2->left_img_.cols, frame2->right_img_.cols);
}