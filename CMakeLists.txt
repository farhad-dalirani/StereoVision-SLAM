cmake_minimum_required(VERSION 2.8)
project(StereoVisionSLAM)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++17 -O3 -g -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#####################################
### dependencies
#####################################

# Suitesparse
include_directories("/usr/include/suitesparse/")

# G2O
Find_Package(G2O REQUIRED)

# Eigen
Find_Package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Sophus
Find_Package(Sophus REQUIRED)

# csparse
Find_Package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# rerun
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# pcl 
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core g2o_types_slam3d g2o_solver_csparse)

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        Sophus::Sophus
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension g2o_types_slam3d
        pthread
        ${CSPARSE_LIBRARY}
        ${GTEST_BOTH_LIBRARIES}
        rerun_sdk
        ${PCL_LIBRARIES}
        fmt
        )



############### source ######################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(app)
