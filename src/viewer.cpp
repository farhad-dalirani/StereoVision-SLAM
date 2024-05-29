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

        // World orgin
        rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP); // Set an up-axis
        rec.log_static(
            "world",
            rerun::Arrows3D::from_vectors({{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}
            ).with_colors({{255, 0, 0}, {0, 255, 0}, {0, 0, 255}})
        );

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
        all_keyframes_ = map_->GetAllKeyFrames();
        active_keyframes_ = map_->GetActiveKeyFrames();
        active_landmarks_ = map_->GetActiveMapPoints();

        // Oreder keyframes id in decreasing order
        std::vector<std::pair<unsigned long, Frame::Ptr>> kf_sort{active_keyframes_.begin(), active_keyframes_.end()};
        std::sort(kf_sort.begin(), kf_sort.end(), 
                [](std::pair<unsigned long, Frame::Ptr> &a, std::pair<unsigned long, Frame::Ptr> &b)
                    {return a.first > b.first;});

        // Use most recent active keyframe id as sequence number for visualization
        rec.set_time_sequence("max_keyframe_id", kf_sort[0].first);

        // Draw all active keyframes
        for(size_t i{0}; i < kf_sort.size(); ++i)
        {
            std::string entity_name = std::string("world/stereosys") + std::to_string(i) + std::string("/cam_left");
            
            if (i != 0)
            {
                Sophus::SE3d Twc0 = kf_sort[0].second->pose_.inverse() * camera_left_->pose_inv_;
                Sophus::SE3d Twci = kf_sort[i].second->pose_.inverse() * camera_left_->pose_inv_;
                Sophus::SE3d Tcic0 = Twci.inverse() * Twc0;
                const Eigen::Vector3f camera_position = Tcic0.translation().cast<float>();
                Eigen::Matrix3f camera_orientation = Tcic0.rotationMatrix().cast<float>();
                rec.log(entity_name,
                        rerun::Transform3D(
                            rerun::Vec3D(camera_position.data()),
                            rerun::Mat3x3(camera_orientation.data()), true)
                    );
            }

            // Left camera pinwhole model
            float fx{camera_left_->fx_};
            float fy{camera_left_->fy_};
            float img_num_rows{kf_sort[i].second->left_img_.rows};
            float img_num_cols{kf_sort[i].second->left_img_.cols};
            rec.log(entity_name,
                    rerun::Pinhole::from_focal_length_and_resolution({fx, fy}, {img_num_cols, img_num_rows}));


            // Draw stereo images of newest keyframe
            if(i == 0)
            {        
                rec.log(entity_name, 
                        rerun::Image(tensor_shape(kf_sort[i].second->left_img_), 
                                    rerun::TensorBuffer::u8(kf_sort[i].second->left_img_)));
            }
        }

        Sophus::SE3d Twc0 = kf_sort[0].second->pose_.inverse() * camera_left_->pose_inv_;
        const Eigen::Vector3f camera_position = Twc0.translation().cast<float>();
        Eigen::Matrix3f camera_orientation = Twc0.rotationMatrix().cast<float>();
        rec.log("world/landmarks",
                rerun::Transform3D(
                    rerun::Vec3D(camera_position.data()),
                    rerun::Mat3x3(camera_orientation.data()), true));
        // Draw active landmarks
        std::vector<Eigen::Vector3f> points3d_vector;
        for(auto iter{active_landmarks_.begin()}; iter != active_landmarks_.end(); iter++)
        {
            points3d_vector.push_back(iter->second->pos_.cast<float>());
        }
        rec.log("world/landmarks", rerun::Points3D(points3d_vector));

        
        
        rec.log("world/path",
                rerun::Transform3D(
                    rerun::Vec3D(camera_position.data()),
                    rerun::Mat3x3(camera_orientation.data()), true));
        std::vector<rerun::datatypes::Vec3D> path;
        for(unsigned i{0}; i < kf_sort[0].first; ++i)
        {   

            Eigen::Vector3f cam_position = (all_keyframes_[i]->pose_.inverse() * camera_left_->pose_).translation().cast<float>();
            path.emplace_back(cam_position.data());
        }
        rec.log("world/path", rerun::LineStrips3D(rerun::LineStrip3D(path)));

    }


}  