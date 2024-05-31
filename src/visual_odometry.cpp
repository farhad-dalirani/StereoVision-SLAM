//
// Created by gaoxiang on 19-5-4.
//
#include "StereoVisionSLAM/visual_odometry.h"
#include <chrono>
#include "StereoVisionSLAM/config.h"
#include "StereoVisionSLAM/slamexception.h"

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
        
        // Create components
        frontend_ = std::make_shared<Frontend>();
        backend_ = std::make_shared<Backend>();
        map_ = std::make_shared<Map>();
        if(Config::Get<int>("visualizer_on") >= 1)
        {
            viewer_ = std::make_shared<Viewer>();
        }

        frontend_->SetBackend(backend_);
        frontend_->SetMap(map_);
        frontend_->SetViewer(viewer_);
        frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));
        
        backend_->SetMap(map_);
        backend_->SetViewer(viewer_);
        backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

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
            viewer_->LogInfo(std::string("VO         : new frame process is running"));
        }

        auto t1 = std::chrono::steady_clock::now();

        // Feed new frame to the stereo visual slam pipeline
        bool success = frontend_->AddFrame(new_frame);

        auto t2 = std::chrono::steady_clock::now();
        auto time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        
        if(viewer_)
        {
            viewer_->LogInfo(std::string("VO        : new frame process cost time ") + 
                         std::to_string(time_used.count()) + 
                         std::string(" seconds"));
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
        
        // Stop other components
        backend_->Stop();
        if(viewer_)
        {
            viewer_->Close();
        }
    } 

    FrontendStatus VisualOdometry::GetFrontendStatus() const
    { 
        return frontend_->GetStatus(); 
    }

}  