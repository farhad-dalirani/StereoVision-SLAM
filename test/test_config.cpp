// test_config.cpp

#include <gtest/gtest.h>
#include "StereoVisionSLAM/config.h"
#include <opencv2/opencv.hpp>
#include <fstream>

class ConfigTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a temporary YAML file for testing
        std::ofstream file("config/test_config.yaml");
        file << "%YAML:1.0\n";
        file << "test_param: 42\n";
        file << "test_float: 3.14\n";
        file << "test_string: hello world\n";
        file.close();
    }

    void TearDown() override {
        // Remove the temporary file after tests
        std::remove("config/test_config.yaml");
    }
};

TEST_F(ConfigTest, SetParameterFile) {
    
    // Attempt to set a non-existent file
    bool result = slam::Config::SetParameterFile("config/non_existent_file.yaml");
    EXPECT_FALSE(result);
    result = slam::Config::SetParameterFile("config/test_config.yaml");
    EXPECT_TRUE(result);
}

TEST_F(ConfigTest, GetIntParameter) {
    slam::Config::SetParameterFile("config/test_config.yaml");
    int value = slam::Config::Get<int>("test_param");
    EXPECT_EQ(value, 42);
}

TEST_F(ConfigTest, GetFloatParameter) {
    slam::Config::SetParameterFile("config/test_config.yaml");
    float value = slam::Config::Get<float>("test_float");
    EXPECT_FLOAT_EQ(value, 3.14);
}

TEST_F(ConfigTest, GetStringParameter) {
    slam::Config::SetParameterFile("config/test_config.yaml");
    std::string value = slam::Config::Get<std::string>("test_string");

    EXPECT_EQ(value, "hello world");
}

