#include <gtest/gtest.h>
#include "StereoVisionSLAM/map.h"

using namespace slam;

// Test fixture for the Map class
class MapTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize any objects required for tests
    }

    void TearDown() override {
        // Clean up any objects created in SetUp()
    }

    // Add any helper functions here
};

// Test case for inserting a keyframe
TEST_F(MapTest, InsertKeyFrameTest) {

    Map map;
    
    Frame::Ptr frame1 = slam::Frame::CreateFrame();
    frame1->SetKeyFrame();

    map.InsertKeyFrame(frame1);

    ASSERT_EQ(map.GetActiveKeyFrames().size(), 1);
    ASSERT_EQ(map.GetAllKeyFrames().size(), 1);

    Frame::Ptr frame2 = slam::Frame::CreateFrame();
    frame2->SetKeyFrame();
    map.InsertKeyFrame(frame2);


    ASSERT_EQ(map.GetActiveKeyFrames().size(), 2);
    ASSERT_EQ(map.GetAllKeyFrames().size(), 2);
}

// Test case for inserting a map point
TEST_F(MapTest, InsertMapPointTest) {
    Map map;
    MapPoint::Ptr map_point = MapPoint::CreateNewMappoint();

    map.InsertMapPoint(map_point);

    ASSERT_EQ(map.GetAllMapPoints().size(), 1);
    ASSERT_EQ(map.GetActiveMapPoints().size(), 1);
}

// Test case for cleaning up the map
TEST_F(MapTest, CleanMapTest) {
    Frame::Ptr frame = slam::Frame::CreateFrame();
    cv::KeyPoint keypoint(100, 200, 2.5);
    Feature::Ptr feature = std::make_shared<slam::Feature>(frame, keypoint);
    
    Map map;
    MapPoint::Ptr map_point = MapPoint::CreateNewMappoint();

    map_point->AddObservation(feature);

    map.InsertMapPoint(map_point);
    ASSERT_EQ(map.GetActiveMapPoints().size(), 1);
    
    map_point->RemoveObservation(feature);
    
    map.CleanMap();


    ASSERT_EQ(map.GetActiveMapPoints().size(), 0);
}


// Test case for removing a old key
TEST_F(MapTest, RemoveOldKeyFrame) {

    Map map;
    
    for(int i{0}; i < 100; i++)
    {
        Frame::Ptr frame1 = slam::Frame::CreateFrame();
        frame1->SetKeyFrame();

        map.InsertKeyFrame(frame1);
    }

    ASSERT_NE(map.GetActiveKeyFrames().size(), map.GetAllKeyFrames().size());
}
