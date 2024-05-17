#include "StereoVisionSLAM/config.h"


namespace slam
{
    Config::~Config()
    {
        // Clos config file
        if (file_.isOpened())
        {
            file_.release();
        }
    }

    bool Config::SetParameterFile(const std::string &filename) 
    {
        // Set a new config file

        if (config_ == nullptr)
        {
            // Create singleton object once
            config_ = std::shared_ptr<Config>(new Config);
        }
        
        // Read config file
        config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
        if (config_->file_.isOpened() == false) 
        {
            std::cout << "parameter file " << filename << " does not exist." << std::endl;
            config_->file_.release();
            return false;
        }

        return true;
    }

    std::shared_ptr<Config> Config::config_ = nullptr;
}