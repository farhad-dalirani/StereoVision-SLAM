//
// Created by gaoxiang on 19-5-4.
//
#include "StereoVisionSLAM/visual_odometry.h"
#include "StereoVisionSLAM/config.h"
#include "StereoVisionSLAM/slamexception.h"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

namespace slam 
{
    VisualOdometry::VisualOdometry(std::string &config_file_path)
    : config_file_path_(config_file_path)
    {
    }

    bool VisualOdometry::initialize()
    {
        /* Initialize different componets of Stereo SLAM pipeline
         * including dataset, map, frontend, backend, viewer */

        // Create config object from config file
        if(Config::SetParameterFile(config_file_path_) == false)
        {
            return false;
        }
        
        // Create Dataset object
        dataset_ = std::make_shared<Dataset>(
                        Dataset(Config::Get<std::string>("dataset_dir"))); 
        
        // Initialize dataset
        if(!dataset_->initialize())
        {
            throw SLAMException("Cannot initialize object to read dataset.");
        }
        
        // Create SLAM Pipeline components
        
        // Frontend
        frontend_ = std::make_shared<Frontend>();
        
        // Backend
        if(Config::Get<int>("backend_on") >= 1)
        {
            backend_ = std::make_shared<Backend>();
        }

        // Loop Closure
        if(Config::Get<int>("loopclosure_on") >= 1)
        {
            loopclosure_ = std::make_shared<LoopClosure>();
        }

        // Map
        map_ = std::make_shared<Map>();
        
        // Visualizer
        if(Config::Get<int>("visualizer_on") >= 1)
        {
            viewer_ = std::make_shared<Viewer>();
        }

        frontend_->SetBackend(backend_);
        frontend_->SetLoopClosure(loopclosure_);
        frontend_->SetMap(map_);
        frontend_->SetViewer(viewer_);
        frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));
        
        if(backend_)
        {
            backend_->SetMap(map_);
            backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));
            if(viewer_)
            {
                backend_->SetViewer(viewer_);
            }
        }

        if(loopclosure_)
        {
            loopclosure_->SetMap(map_);
            loopclosure_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));
            loopclosure_->SetFrontend(frontend_);
            if(backend_)
            {
                loopclosure_->SetBackend(backend_);
            }
            if(viewer_)
            {
                loopclosure_->SetViewer(viewer_);
            }
        }

        if(viewer_)
        {
            viewer_->SetMap(map_);
            viewer_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));
        }

        return true;
    }
    
    bool VisualOdometry::step()
    {
        // Processes the next frame in the stereo visual odometry pipeline
        
        // Read next frame
        Frame::Ptr new_frame{dataset_->NextFrame()};
        if(new_frame == nullptr)
        {
            return false;
        }

        if(viewer_)
        {
            viewer_->LogInfo(
                std::string("VO         : new frame process is running"), "vo");
        }

        auto t1 = std::chrono::steady_clock::now();

        bool success;
        {
            if(loopclosure_)
            {
                std::unique_lock<std::mutex> lck(loopclosure_->loop_closure_upd_);
                // Feed new frame to the stereo visual slam pipeline
                success = frontend_->AddFrame(new_frame);
            }
            else
            {
                // Feed new frame to the stereo visual slam pipeline
                success = frontend_->AddFrame(new_frame);
            }
        }

        auto t2 = std::chrono::steady_clock::now();
        auto time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        
        if(viewer_)
        {
            viewer_->LogInfo(std::string("VO        : new frame process cost time ") + 
                         std::to_string(time_used.count()) + 
                         std::string(" seconds"),
                         "vo");
        }

        return success;
    }

    void VisualOdometry::run()
    {
        /* Process whole input image sequece with 
         *  stereo visual slam pipline */

        // Feed every frame of sequence to SLAM pipleline 
        while (true)
        {          
            if(step() == false)
            {
                // If there no other frame in sequence, stop
                break;
            }
        }

        // Stop components of stereo visual slam pipeline
        if(loopclosure_)
        {
            loopclosure_->Stop();
        }
        
        if(backend_)
        {
            backend_->Stop();
        }

        if(viewer_)
        {
            viewer_->Close();
        }

        // Save landmarks and keyframes in file
        saveSLAMOutputInFile();
    } 

    FrontendStatus VisualOdometry::GetFrontendStatus() const
    { 
        return frontend_->GetStatus(); 
    }

    void VisualOdometry::saveSLAMOutputInFile()
    {
        // Save output of Stereo Visual SLAM (landmarks and keyframe poses) in file

        // Output directory
        std::string output_dir = Config::Get<std::string>("output_dir"); 

        // Check if folder path exists
        if (not(std::filesystem::exists(output_dir) && std::filesystem::is_directory(output_dir)))
        {
            std::cout << "Folder does not exist: " << output_dir << std::endl;
            return;
        }

        // Create a folder with the current time as its name
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
        std::string new_folder_name = ss.str();
        std::filesystem::path new_folder_path = std::filesystem::path(output_dir) / new_folder_name;

        if (not std::filesystem::create_directory(new_folder_path)) 
        {
            std::cerr << "Failed to create folder: " << new_folder_path << std::endl;
            return;
        }

        // Save landmarks in .pcl file
        std::filesystem::path landmark_file_path = new_folder_path / "landmarks.pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for(auto &id_point_pair_i: map_->GetAllMapPoints())
        {
            MapPoint::Ptr mappoint_i = id_point_pair_i.second;

            pcl::PointXYZ point;
            point.x = mappoint_i->pos_(0);
            point.y = mappoint_i->pos_(1);
            point.z = mappoint_i->pos_(2);
            cloud->points.push_back(point);
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;
        if (pcl::io::savePCDFileASCII(landmark_file_path, *cloud) == -1) 
        {
            throw SLAMException("Could not write file");
        }
        std::cout << "Saved " << cloud->points.size() << " landmarks to " << landmark_file_path << std::endl;

        /* 
        Save keyframe poses in file, File format:

        dataset_dir
        left_cam_index_in_stereo_system
        left_camera_image_id_1, tranformation_matrix_of_frame_1 (from world to stereosystem coordinate)
        left_camera_image_id_2, tranformation_matrix_of_frame_2 (from world to stereosystem coordinate)
        .
        .
        .
        */
        std::filesystem::path keyframes_file_path = new_folder_path / "keyframes.txt";
        std::ofstream kp_file(keyframes_file_path);

        // Write data sequence dir path
        kp_file << dataset_->GetDataDir() << std::endl;
        // Write left camera index among cameras of stereo system
        kp_file << dataset_->GetLeftCamIndex() << std::endl;
        
        // Sort according apearance in time
        Map::KeyframesType all_kf_unordered_map = map_->GetAllKeyFrames();
        std::vector<std::pair<unsigned long, Frame::Ptr>> kf_sorted_vect(
                                                            all_kf_unordered_map.begin(), 
                                                            all_kf_unordered_map.end());
        sort(kf_sorted_vect.begin(), kf_sorted_vect.end(), [](
                    const std::pair<unsigned long, Frame::Ptr>& a, 
                    const std::pair<unsigned long, Frame::Ptr>& b) 
                    {
                        return a.first < b.first;
                    });

        // For each keyframe
        for(auto &id_keyframe_i: kf_sorted_vect)
        {
            Frame::Ptr keyframe_i = id_keyframe_i.second;

            // Write image id of left camera image of stereosystem
            kp_file << keyframe_i->id_ << " ";

            // Transformation matrix 
            Eigen::Matrix4d Tcw = keyframe_i->Pose().matrix();
            for(int i{0}; i < 3; i++)
            {
                for(int j{0}; j < 4; j++)
                {
                    kp_file << Tcw(i, j);
                    
                    if(i * 4 + j < 11)
                    {
                        kp_file << " ";
                    }
                    else
                    {
                        kp_file << std::endl;
                    }
                }
            }


        }


    }

}