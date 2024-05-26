#include "StereoVisionSLAM/visual_odometry.h"

int main(int argc, char **argv) 
{
    
    std::string config_file_path = "./config/default.yaml";
    
    slam::VisualOdometry::Ptr vo = std::make_shared<slam::VisualOdometry>(config_file_path);
    
    assert(vo->initialize() == true);
    
    vo->run();

    return 0;
}
