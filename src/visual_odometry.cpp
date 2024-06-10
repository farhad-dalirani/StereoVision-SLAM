//
// Created by gaoxiang on 19-5-4.
//
#include "StereoVisionSLAM/visual_odometry.h"
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

        // Feed new frame to the stereo visual slam pipeline
        bool success = frontend_->AddFrame(new_frame);

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
    } 

    FrontendStatus VisualOdometry::GetFrontendStatus() const
    { 
        return frontend_->GetStatus(); 
    }

}  