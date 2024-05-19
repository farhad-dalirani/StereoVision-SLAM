#include <gtest/gtest.h>
#include<StereoVisionSLAM/common_include.h>
#include<StereoVisionSLAM/algorithm.h>

TEST(MyslamTest, Triangulation) {
    Eigen::Vector3d pt_world(30, 20, 10), pt_world_estimated;
    std::vector<Sophus::SE3d> poses{
            Sophus::SE3d(Eigen::Quaterniond(0, 0, 0, 1), Eigen::Vector3d(0, 0, 0)),
            Sophus::SE3d(Eigen::Quaterniond(0, 0, 0, 1), Eigen::Vector3d(0, -10, 0)),
            Sophus::SE3d(Eigen::Quaterniond(0, 0, 0, 1), Eigen::Vector3d(0, 10, 0)),
    };
    std::vector<Eigen::Vector3d> points;
    for (size_t i = 0; i < poses.size(); ++i) {
        Eigen::Vector3d pc = poses[i] * pt_world;
        pc /= pc[2];
        points.push_back(pc);
    }

    EXPECT_TRUE(slam::triangulation(poses, points, pt_world_estimated));
    EXPECT_NEAR(pt_world[0], pt_world_estimated[0], 0.01);
    EXPECT_NEAR(pt_world[1], pt_world_estimated[1], 0.01);
    EXPECT_NEAR(pt_world[2], pt_world_estimated[2], 0.01);
}

TEST(MyslamTest, toVec2Function){
    cv::Point2f p(1,2);
    Eigen::Vector2d out = slam::toVec2(p);

    EXPECT_EQ(out(0), p.x);
    EXPECT_EQ(out(1), p.y);
}