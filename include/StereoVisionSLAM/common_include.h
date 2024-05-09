#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// C++ STL
#include <iostream>
#include <string>
#include <vector>
#include <memory>

// Eigen Library for for linear algebra,
// matrix and vector operations, geometrical transformations, ...
#include <Eigen/Core>
#include <Eigen/Geometry>

// Sophus Library for  Lie Groups and Algebra operations
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

// OpenCV 
#include <opencv2/core/core.hpp>

// custom exception
#include "StereoVisionSLAM/slamexception.h"

#endif 