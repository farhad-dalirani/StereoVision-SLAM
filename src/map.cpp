#include<StereoVisionSLAM/map.h>

namespace slam
{

    void Map::CleanMap()
    {
        // Clean up the points with zero observations from 
        // active landmarks of the map

        int cnt_removed_active_lan{0};
        for(auto iter{active_landmarks_.begin()}; iter != active_landmarks_.end();)
        {
            if(iter->second->observed_times_ == 0)
            {
                iter = active_landmarks_.erase(iter);
                cnt_removed_active_lan++;
            }
            else
            {
                iter++;
            }
        }
        std::cout << "Removed " << cnt_removed_active_lan << " active landmarks." << std::endl;
    }

    void Map::InsertKeyFrame(Frame::Ptr frame)
    {
        current_frame_ = frame;

        // add/update frame to both keyframes and active keyframes
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;

        // if number of active keyframes are larger that the specified
        // value, remove one of them
        if(active_keyframes_.size() > num_active_keyframes_)
        {
            RemoveOldKeyframe();
        }
    }

    void Map::InsertMapPoint(MapPoint::Ptr map_point)
    {
        // Add/update a new point to the map
        landmarks_[map_point->id_] = map_point;
        active_landmarks_[map_point->id_] = map_point;
    }

    void Map::RemoveOldKeyframe()
    {
        // Set old keyframes to inactive state

        if(current_frame_ == nullptr)
        {
            return;
        }

        // Find the nearest and farthest two active keyframes
        // to the current frame
        double max_dis{0}, min_dis{999999};
        unsigned long max_kf_id{0}, min_kf_id{0};

        // Transformation from camera coordinate system
        // to world coordinate system (map)
        Sophus::SE3d Twc = current_frame_->Pose().inverse();

        for(const auto &kf: active_keyframes_)
        {
            if (kf.second == current_frame_)
            {
                // Do not compare current frame with itself
                continue;
            }

            // Calculates the distance between frames. First, the 
            // transformation between two frames is calculated. Then,
            // the vector representation of the transformation matrix 
            // in Lie algebra is computed by taking the log. Finally,
            // the norm of the transformation vector is used to quantify 
            // the distance between the frames.
            double dis = (kf.second->Pose() * Twc).log().norm();

            if (dis > max_dis)
            {
                max_dis = dis;
                max_kf_id = kf.first;
            }
            if (dis < min_dis)
            {
                min_dis = dis;
                min_kf_id = kf.first;
            }
        }
        
        // hyper-parameter
        const double min_dis_th{0.2};
        Frame::Ptr frame_to_remove{nullptr};
        // if closest active frame to the current active keyframe
        // is so similar in pose, remove it, otherwise remove the 
        // active keyframe with largest pose difference
        if(min_dis < min_dis_th)
        {
            frame_to_remove = active_keyframes_.at(min_kf_id);
        }
        else
        {
            frame_to_remove = active_keyframes_.at(max_kf_id);
        }

        std::cout << "Remove keyframe " << frame_to_remove->keyframe_id_ << std::endl;

        // remove keyframe from active key frames
        active_keyframes_.erase(frame_to_remove->keyframe_id_);

        // remove links between map points(landmarks) and keypoints features
        // of removed key frame 
        for(auto feat_ptr: frame_to_remove->feature_left_)
        {
            MapPoint::Ptr mp = feat_ptr->map_point_.lock();
            if(mp)
            {
                mp->RemoveObservation(feat_ptr);
            }
        }
        for(auto feat_ptr: frame_to_remove->feature_right_)
        {
            if (feat_ptr == nullptr) 
            {
                // If a keypoint feature from the left camera is not tracked 
                // in the right camera, the corresponding feature point will be null.
                continue;
            }
            MapPoint::Ptr mp = feat_ptr->map_point_.lock();
            if(mp)
            {
                mp->RemoveObservation(feat_ptr);
            }
        }

        CleanMap();
    }


    Map::LandmarksType Map::GetAllMapPoints()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }
    
    
    Map::KeyframesType Map::GetAllKeyFrames()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }


    Map::LandmarksType Map::GetActiveMapPoints()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }


    Map::KeyframesType Map::GetActiveKeyFrames() 
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

}