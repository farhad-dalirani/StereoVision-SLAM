#include "StereoVisionSLAM/dense_reconstruction.h"

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
        std::cerr << "Error: No input path specified for 3D dense reconstruction config file."
                  << std::endl;
        return 1;
    }
    
    std::cout << "Outputs are visualized in Rerun Viewer." << std::endl;

    // Creat and initialize Dense 3D reconstruction pipeline
    slam::DenseReconstruction dense_rec = slam::DenseReconstruction(config_file_path);
    dense_rec.Initialize();

    // Create Dense Reconstruction by using output of SLAM pipeline
    dense_rec.DenseReconstruct();


    std::cout << "Finished." << std::endl;

    return 0;
}
