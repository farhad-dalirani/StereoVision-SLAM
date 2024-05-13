#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "StereoVisionSLAM/common_include.h"
#include "StereoVisionSLAM/feature.h"

namespace slam
{

    // Avoid circular dependency error
    class Frame;
    class Feature;

    class MapPoint
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<MapPoint> Ptr;

            unsigned long id_{0};
            bool is_outlier_{false};
            // 3d Poisiton of point in world coordinate
            Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();
            // To control access race amont threads
            std::mutex data_mutex_;
            // Number of times being observed by feature matching algorithm
            int observed_times_{0};
            // Keypoint features corresponding to the point in different frames  
            std::list<std::weak_ptr<Feature>> observations_;

            MapPoint(){}
            MapPoint(unsigned long id, const Eigen::Vector3d &pos);
            
            // return 3d position of map point in world coordinate
            Eigen::Vector3d Pos();
            
            void SetPos(const Eigen::Vector3d &pos);
            
            void AddObservation(Feature::Ptr feature);

            void RemoveObservation(Feature::Ptr feature);

            // Factory construction pattern, assigning IDs
            static Ptr CreateNewMappoint();

    };

}

#endif