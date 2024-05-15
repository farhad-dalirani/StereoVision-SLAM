#include <gtest/gtest.h>
#include<StereoVisionSLAM/mappoint.h>

using namespace slam;

TEST(MapPointTest, PosTest) 
{
    Eigen::Vector3d pos(1.0, 2.0, 3.0);
    MapPoint map_point(1, pos);

    Eigen::Vector3d retrieved_pos = map_point.Pos();

    for(int i{0}; i < 3; i++)
        ASSERT_EQ(pos[i], retrieved_pos[i]);
}

TEST(MapPointTest, SetPosTest) 
{
    Eigen::Vector3d pos(1.0, 2.0, 3.0);
    MapPoint map_point(1, pos);

    Eigen::Vector3d new_pos(4.0, 5.0, 6.0);
    map_point.SetPos(new_pos);

    Eigen::Vector3d retrieved_pos = map_point.Pos();

    for(int i{0}; i < 3; i++)
        ASSERT_EQ(new_pos[i], retrieved_pos[i]);
}

TEST(MapPointTest, CreateMapPoint)
{
    MapPoint::Ptr map_point = MapPoint::CreateNewMappoint();
    ASSERT_EQ(map_point->id_, 0);

    MapPoint::Ptr map_point2 = MapPoint::CreateNewMappoint();
    ASSERT_EQ(map_point2->id_, 1);
}

TEST(MapPointTest, AddRemoveObservationTest) 
{
    MapPoint::Ptr map_point = MapPoint::CreateNewMappoint();
    Feature::Ptr feature = std::make_shared<Feature>();

    map_point->AddObservation(feature);

    ASSERT_EQ(map_point->observed_times_, 1);

    map_point->RemoveObservation(feature);

    ASSERT_EQ(map_point->observed_times_, 0);
}

