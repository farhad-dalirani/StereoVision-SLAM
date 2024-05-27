#ifndef RERUNCOLLECTIONADAPTERS_H
#define RERUNCOLLECTIONADAPTERS_H

#include "StereoVisionSLAM/common_include.h"

#include <rerun.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/core.hpp>

#include <cassert>


template <>
struct rerun::CollectionAdapter<uint8_t, cv::Mat> 
{
    /* Adapters to borrow an OpenCV image into Rerun
     * images without copying */
    
    Collection<uint8_t> operator()(const cv::Mat& img) 
    {
        // Borrow for non-temporary.
        
        assert("OpenCV matrix expected have bit depth CV_U8" && CV_MAT_DEPTH(img.type()) == CV_8U);
        return Collection<uint8_t>::borrow(img.data, img.total() * img.channels());
    }
    
    Collection<uint8_t> operator()(cv::Mat&& img) 
    {
        /* Do a full copy for temporaries (otherwise the data
         * might be deleted when the temporary is destroyed). */
        
        assert("OpenCV matrix expected have bit depth CV_U8" && CV_MAT_DEPTH(img.type()) == CV_8U);
        std::vector<uint8_t> img_vec(img.total() * img.channels());
        img_vec.assign(img.data, img.data + img.total() * img.channels());
        return Collection<uint8_t>::take_ownership(std::move(img_vec));
    }
};


template <>
struct rerun::CollectionAdapter<rerun::Position3D, std::vector<Eigen::Vector3f>> 
{
    /* Adapters to log eigen vectors as rerun positions*/

    Collection<rerun::Position3D> operator()(const std::vector<Eigen::Vector3f>& container) 
    {
        // Borrow for non-temporary.
        return Collection<rerun::Position3D>::borrow(container.data(), container.size());
    }

    Collection<rerun::Position3D> operator()(std::vector<Eigen::Vector3f>&& container) 
    {
        /* Do a full copy for temporaries (otherwise the data
         * might be deleted when the temporary is destroyed). */
        std::vector<rerun::Position3D> positions(container.size());
        memcpy(positions.data(), container.data(), container.size() * sizeof(Eigen::Vector3f));
        return Collection<rerun::Position3D>::take_ownership(std::move(positions));
    }
};

template <>
struct rerun::CollectionAdapter<rerun::Position3D, Eigen::Matrix3Xf> 
{
    /* Adapters so we can log an eigen matrix as rerun positions */
    
    // Sanity check that this is binary compatible.
    static_assert(
        sizeof(rerun::Position3D) == sizeof(Eigen::Matrix3Xf::Scalar) * Eigen::Matrix3Xf::RowsAtCompileTime
    );

    Collection<rerun::Position3D> operator()(const Eigen::Matrix3Xf& matrix) 
    {
        // Borrow for non-temporary.
        static_assert(alignof(rerun::Position3D) <= alignof(Eigen::Matrix3Xf::Scalar));
        return Collection<rerun::Position3D>::borrow(
            // Cast to void because otherwise Rerun will try to do above sanity checks with the wrong type (scalar).
            reinterpret_cast<const void*>(matrix.data()),
            matrix.cols()
        );
    }

    Collection<rerun::Position3D> operator()(Eigen::Matrix3Xf&& matrix) 
    {
        /* Do a full copy for temporaries (otherwise the 
         * data might be deleted when the temporary is destroyed). */
        std::vector<rerun::Position3D> positions(matrix.cols());
        memcpy(positions.data(), matrix.data(), matrix.size() * sizeof(rerun::Position3D));
        return Collection<rerun::Position3D>::take_ownership(std::move(positions));
    }
};

rerun::Collection<rerun::TensorDimension> tensor_shape(const cv::Mat& img)
{
        return {img.rows, img.cols, img.channels()};
}

#endif