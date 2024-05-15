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

            Map(){}

            

        
        private:
            std::mutex data_mutex_;
            
            // All landmartks; Hashtable (id-landmark(map point))
            LandmarksType landmarks_;
            // Active landmartks; Hashtable (id-landmark(map point))
            LandmarksType active_landmarks_;
            
            // All key frames; Hashtable (id-frame)
            KeyframesType keyframes_;
            // Active key frames; Hashtable (id-frame)
            KeyframesType active_keyframes_;

            Frame::Ptr current_frame_ = nullptr;

            // Hyper-parameter: number of active frames
            int num_active_keyframes_{7};
    };

}

#endif