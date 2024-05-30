#include "StereoVisionSLAM/visual_odometry.h"

int main(int argc, char **argv) 
{
    
    std::string config_file_path = "./config/default.yaml";
    
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
