#ifndef SLAM_G2O_TYPES_H
#define SLAM_G2O_TYPES_H

#include "StereoVisionSLAM/common_include.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace slam
{
    /* Definitions of custom vertices and edges for g2o graph-based 
     * non-linear optimization used in the frontend and backend.
     */

    class VertexPose: public g2o::BaseVertex<6, Sophus::SE3d>
    {
        // Pose vertex (6D pose vector in lie algebra 3 rotation + 3 translation)
        
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            virtual void setToOriginImpl() override
            {
                // Reset to initail value for paramers
                _estimate = Sophus::SE3d();
            }

            virtual void oplusImpl(const double *update) override
            {
                /* 
                 * Update parameters according to the calculated update value.

                 * This involves left multiplication on SE(3) (Left Perturbation).
                
                 * Refer to section 3.3.5 [Derivative on SE(3)] in the book
                 * "Introduction to Visual SLAM: From Theory to Practice" 
                 * by Xiang Gao and Tao Zhang.
                 */
                
                // Convert double array to Eigen vector
                Eigen::Matrix<double, 6, 1> update_vec;
                update_vec <<  update[0], update[1], update[2], update[3], update[4], update[5];

                /* Convert the update vector from its Lie algebra representation 
                 * to a Lie group (transformation matrix), and then left multiply it 
                 * with the current estimate of the parameters. */
                _estimate = Sophus::SE3d::exp(update_vec) * _estimate;
            }

            // For reading and writing into file, not implemented
            virtual bool read(std::istream &in) override { return true; }
            virtual bool write(std::ostream &out) const override { return true; }
    };


    class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
    {
        /*
            Edge definition for the g2o graph, used for optimizing the current frame's
            pose in the frontend stage.
            
            This class represents an edge in a graph optimization problem that uses
            the projection of a 3D point onto current frame left camera 2D image plane.
             
            The difference between the projection of the 3D point and its
            corresponding 2D keypoint feature is used to calculate the error
            of the estimated parameters (pose of the current frame).
        */

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            EdgeProjectionPoseOnly(const Eigen::Vector3d &pos,
                                   const Eigen::Matrix3d &K)
            : _pos3d(pos), _K(K)
            {
            }

            virtual void computeError() override
            {
                // Obtain current estimate of parameters (pose of current frame)
                const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
                Sophus::SE3d T = v->estimate();
                
                /* Project landmark from world coordinate to 
                 * current frame left image */
                Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
                pos_pixel /= pos_pixel[2];

                // calculate error of projected point and its corresponding feature
                _error = _measurement - pos_pixel.head<2>();
            }

            virtual void linearizeOplus() override 
            {
                /* Computes the Jacobian of the error function
                 * with respect to parameters (pose vector)
                 *
                 * Gradient calculation similar to
                 * 6.7.3 [Solve PnP by Minimizing the Reprojection Error]
                 * in the book "Introduction to Visual SLAM: From Theory to Practice" 
                 * by Xiang Gao and Tao Zhang. */

                // Obtain current estimate of parameters (pose of current frame)
                const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
                Sophus::SE3d T = v->estimate();

                /* Convert 3D point from world coordinate
                 * to left camera coordinate */
                Eigen::Vector3d pos_cam = T * _pos3d;

                double fx = _K(0, 0);
                double fy = _K(1, 1);
                double X = pos_cam[0];
                double Y = pos_cam[1];
                double Z = pos_cam[2];
                double Zinv = 1.0 / (Z + 1e-18);
                double Zinv2 = Zinv * Zinv;
                
                // Calculate Jacobian
                _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                    -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                    fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                    -fy * X * Zinv;
            }

            // For reading and writing into file, not implemented
            virtual bool read(std::istream &in) override { return true; }
            virtual bool write(std::ostream &out) const override { return true; }

         private:
            // Edge attribute, 3D point (landmark) in map coordinate system
            Eigen::Vector3d _pos3d;
            // Edge attribute,Intrinsic matrix of stereo system left camera
            Eigen::Matrix3d _K;
    };


}

#endif