#include<StereoVisionSLAM/mappoint.h>

namespace slam
{
    MapPoint::MapPoint(unsigned long id, const Eigen::Vector3d &pos)
    : id_(id), pos_(pos){}


    Eigen::Vector3d MapPoint::Pos()
    {
        // Return 3d position of map point in world coordinate
        std::unique_lock<std::mutex> lck(data_mutex_);
        return pos_;
    }

    void MapPoint::SetPos(const Eigen::Vector3d &pos)
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        pos_ = pos;
    }

    void MapPoint::AddObservation(Feature::Ptr feature)
    {
        /* Registers a 2D keypoint feature that correlates with
         * this 3D point in the world coordinate system
         * Ensures thread safety with a unique lock */
        std::unique_lock<std::mutex> lck(data_mutex_);
        observations_.push_back(feature);
        observed_times_++;

        // Set first valid keypoint feature that observe the map point
        if((first_valid_obs_.expired()) and (feature->outlier_==false))
        {
            first_valid_obs_ = feature;
        }
    }

    void MapPoint::RemoveObservation(Feature::Ptr feature)
    {
        // If provided feature found in list, delete it from list
        
        std::unique_lock<std::mutex> lck(data_mutex_);

        for(auto iter{observations_.begin()}; iter != observations_.end(); iter++)
        {
            if(iter->lock() == feature)
            {
                observations_.erase(iter);
                
                if(feature->outlier_ == true)
                {
                    feature->map_point_.reset();
                }

                observed_times_--;
                break;
            } 
        }

        /* If first valid keypoint feature that observe the map point is no
         * longer valid, select a new one */
        if((first_valid_obs_.lock() == feature) and (feature->outlier_ == true))
        {
            for(auto iter{observations_.begin()}; iter != observations_.end(); iter++)
            {
                Feature::Ptr feat_i = iter->lock();
                if(feat_i)
                {
                    if((feat_i->outlier_ == false) and (feat_i->map_point_.lock()))
                    {
                        first_valid_obs_ = feat_i;
                        break;
                    }
                }
            }
        }

    }
    
    std::list<std::weak_ptr<Feature>> MapPoint::GetObs()
    {
        /* Get 2d keypoint features associated to the map point 
         * across different frames */
        std::unique_lock<std::mutex> lck(data_mutex_);
        return observations_;
    } 

    MapPoint::Ptr MapPoint::CreateNewMappoint()
    {
        // Function to create a new MapPoint object

        static unsigned long factory_id{0};
        
        MapPoint::Ptr new_mappoint = std::make_shared<MapPoint>();
        new_mappoint->id_ = factory_id++;

        return new_mappoint;
    }

}
