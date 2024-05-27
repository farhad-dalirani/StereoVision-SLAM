#include "StereoVisionSLAM/viewer.h"
#include "StereoVisionSLAM/feature.h"
#include "StereoVisionSLAM/frame.h"
#include "StereoVisionSLAM/collection_adapters.h"
#include <opencv2/opencv.hpp>

namespace slam 
{
    void Viewer::SetMap(Map::Ptr map) 
    {
        map_ = map; 
    }

    void Viewer::SetCameras(Camera::Ptr cameraLeft, Camera::Ptr cameraRight)
    {
        camera_left_ = cameraLeft;
        camera_right_ = cameraRight;
    }

    Viewer::Viewer() 
    {
        rec.spawn().exit_on_failure();
    }

    void Viewer::Close() 
    {
    }

    void Viewer::AddCurrentFrame(Frame::Ptr current_frame) 
    {
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        current_frame_ = current_frame;   
    }

    void Viewer::UpdateMap() 
    {
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        assert(map_ != nullptr);
        
        // Get active landmarks and keypoints
        active_keyframes_ = map_->GetActiveKeyFrames();
        active_landmarks_ = map_->GetActiveMapPoints();

        // Obtain most recent active keyframe id
        unsigned long max_keyframe_id_{0};
        for(auto iter{active_keyframes_.begin()}; iter != active_keyframes_.end(); iter++)
        {
            if (iter->second->keyframe_id_ > max_keyframe_id_)
            {
                max_keyframe_id_ = iter->second->keyframe_id_;
            }
        }

        // // Use most recent active keyframe id as sequence number for visualization
        rec.set_time_sequence("max_keyframe_id", max_keyframe_id_);
        
        // Draw active landmarks
        std::vector<Eigen::Vector3f> points3d_vector;
        for(auto iter{active_landmarks_.begin()}; iter != active_landmarks_.end(); iter++)
        {
            points3d_vector.push_back(iter->second->pos_.cast<float>());
        }
        rec.log("world/landmarks", rerun::Points3D(points3d_vector));

        // left camera pinwhole model
        double fx{camera_left_->fx_};
        double fy{camera_left_->fy_};
        double img_num_rows{current_frame_->left_img_.rows};
        double img_num_cols{current_frame_->left_img_.cols};
        rec.log(
            "world/left",
            rerun::Pinhole::from_focal_length_and_resolution({fx, fy}, {img_num_cols, img_num_rows})
        );

        // Left camera pose 
        Sophus::SE3d Twc = current_frame_->pose_.inverse();
        const Eigen::Vector3f camera_position = Twc.translation().cast<float>();
        Eigen::Matrix3f camera_orientation = Twc.rotationMatrix().cast<float>();
    
        rec.log(
            "world/left",
            rerun::Transform3D(
                rerun::Vec3D(camera_position.data()),
                rerun::Mat3x3(camera_orientation.data())
            )
        );

        // Draw stereo images of current frame        
        rec.log("world/left", rerun::Image(tensor_shape(current_frame_->left_img_), 
                              rerun::TensorBuffer::u8(current_frame_->left_img_)));
                              
        rec.log("world/right", rerun::Image(tensor_shape(current_frame_->right_img_), 
                              rerun::TensorBuffer::u8(current_frame_->right_img_)));


    }


}  