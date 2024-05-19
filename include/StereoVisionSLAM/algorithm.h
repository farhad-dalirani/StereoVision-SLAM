#ifndef ALGORITHM_H
#define ALGORITHM_H

#include<StereoVisionSLAM/common_include.h>


namespace slam
{

    inline bool triangulation(const std::vector<Sophus::SE3d> &poses,
                              const std::vector<Eigen::Vector3d> &points,
                              Eigen::Vector3d &pt_world)
    {
        /*
        @param poses: pose of cameras in stereo vision system
        @param points: Corresponding points in the left and right 
                       camera normal planes.
        @param pt_world: Output point in 3d coordinate of stereo system

            Assume there are two cameras (for more cameras is similar):
        
            point1 or p1 = [x1, y1, 1]
            point2 or p2 = [x2, y2, 1]

            pose1 or P1 = [r1; r2; r3] , p1 is 3 * 4, ri is a row  
            pose2 or P2 = [q1; q2; q3] , p2 is 3 * 4, qi is a row
            
            pw is the 3d point corresponding two p1 and p2

            P1 * pw = p1  (project point in world into camera 1)
            P2 * pw = p2  (project point in world into camera 2)  

            P1 * pw = p1 =>     (1) r1 * w = x1
                                (2) r2 * w = y1
                                (3) r3 * w =  1

            P2 * pw = p2 =>     (4) q1 * w = x2
                                (5) q2 * w = y2
                                (6) q3 * w =  1

            (1),(3): r1 * w = x1 => r1 * w = x1 * 1 => r1 * w = x1 * r3 * w
                     => x1 * r3 * w - r1 * w = 0 
                     => (x1 * r3 - r1) * w = 0 
                    
            similarly
            (1),(3): (x1 * r3 - r1) * w = 0
            (2),(3): (y1 * r3 - r2) * w = 0   
            (4),(5): (x2 * q3 - q1) * w = 0
            (5),(6): (y2 * q3 - q2) * w = 0 

            It is a linear system: A * w = 0, solution can be obtained by SVD.    
       */

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A(2 * poses.size(), 4);
        Eigen::Matrix<double, Eigen::Dynamic, 1> b(2 * poses.size());

        // Create A matrix and b vector of linear system
        b.setZero();
        for(size_t i{0}; i < poses.size(); ++i)
        {
            // get extrinsic matrix
            Eigen::Matrix<double, 3, 4> m= poses[i].matrix3x4();

            A.block<1,4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
            A.block<1,4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
        }

        // Solve linear equation
        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

        // In perfect situation third sigular value is zero. However, due to
        // noise it should be near zero and much smaller than second sigular 
        // value.
        if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) 
        {   
            // Calculation good, keep it
            return true;
        }
        // Calculation bad, discard it
        return false;
    }

    inline Eigen::Vector2d toVec2(const cv::Point2f p)
    {
        // Convert opencv 2d vector to Eigen 2d vector
        return Eigen::Vector2d(p.x, p.y);
    }
}


#endif