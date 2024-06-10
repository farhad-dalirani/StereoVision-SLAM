#ifndef MAP_H
#define MAP_H

#include "StereoVisionSLAM/common_include.h"
#include "StereoVisionSLAM/mappoint.h"
#include "StereoVisionSLAM/frame.h"

namespace slam
{
    class Map
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Map> Ptr;
            typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
            typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

            Map();

            // Clean up the points with zero observations from active
            // landmarks of the map
            void CleanMap();
            // Remove a landmark from map
            void RemoveLandmark(MapPoint::Ptr landmark);

            // Add new keyframe
            void InsertKeyFrame(Frame::Ptr frame);

            // Add/update a new point to the map
            void InsertMapPoint(MapPoint::Ptr map_point);

            LandmarksType GetAllMapPoints();
            KeyframesType GetAllKeyFrames();
            LandmarksType GetActiveMapPoints();
            KeyframesType GetActiveKeyFrames();
            

        private:
            std::mutex data_mutex_;
            
            // All landmartks; Hashtable (id-landmark(map point))
            LandmarksType landmarks_;
            // Active landmartks; Hashtable (id-landmark(map point))
            LandmarksType active_landmarks_;
            
            // All key frames; Hashtable (id-keyframe)
            KeyframesType keyframes_;
            // Active key frames; Hashtable (id-keyframe)
            KeyframesType active_keyframes_;

            Frame::Ptr current_frame_{nullptr};

            // Hyper-parameter: number of active keyframes
            int num_active_keyframes_{9};

            // Set old keyframes to inactive state
            void RemoveOldKeyframe();
    };

}

#endif