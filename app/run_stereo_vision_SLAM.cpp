#include "StereoVisionSLAM/visual_odometry.h"

int main(int argc, char **argv) 
{
    // Config file path
    std::string config_file_path;
    if (argc > 1) 
    {
        config_file_path = argv[1];
    } 
    else 
    {
        config_file_path = "./config/stereo_slam_configs/default.yaml";
    }
    
    std::cout << "Outputs are visualized in Rerun Viewer." << std::endl;

    // Create Stereo Visual SLAM Pipeline
    std::unique_ptr<slam::VisualOdometry> vo = std::make_unique<slam::VisualOdometry>(config_file_path);
    
    // Initialize SLAM
    assert(vo->initialize() == true);
    
    // Process all frames in input sequence
    vo->run();

    // Deallocated SLAM pipeline
    vo.reset();

    std::cout << "Finished." << std::endl;

    return 0;
}
