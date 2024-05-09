#include<StereoVisionSLAM/common_include.h>
#include<StereoVisionSLAM/camera.h>
#include<StereoVisionSLAM/dataset.h>

int main()
{
    std::string dataset_path_str{"./data/dataset/sequences/00"};
    slam::Dataset ds(dataset_path_str);

    ds.initialize();

    return 0;
}