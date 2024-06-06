#ifndef LOOPCLOSURE_H
#define LOOPCLOSURE_H

#include "StereoVisionSLAM/common_include.h"
#include "StereoVisionSLAM/map.h"
#include "StereoVisionSLAM/camera.h"
#include "StereoVisionSLAM/viewer.h"
#include <opencv2/dnn.hpp>

namespace slam
{
    
    class LoopClosure
    {
        /*
        */

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<LoopClosure> Ptr;

            // Initialize loop closure pipeline
            LoopClosure();
            // Close loop closure optimization
            void Stop(); 
            void SetMap(std::shared_ptr<Map> map);
            void SetViewer(std::shared_ptr<Viewer> viewer);
            void SetCameras(Camera::Ptr left, Camera::Ptr right);
            // Add a new keyframe to a queue to be process by loop closure pipeline
            void AddNewKeyFrame(Frame::Ptr new_keyframe);
            
        private:
            // Create deep neural netwrok to map a image to a feature vector 
            void InitialFeatureExtractorNetwork();
            // Extract feature representation of image by the backbone
            void ExtractFeatureVec(Frame::Ptr frame);
            // Calculate similarity score of deep representation of two images
            float SimilarityScore(const Eigen::Matrix<float, 1280, 1> &a, 
                                    const Eigen::Matrix<float, 1280, 1> &b);
            // Return True if there are keyframes waiting to be check for loop
            bool IsKeyframeInWaitingList();
            // Select a keyframe to process by loop closure pipeline
            void SetCurrentKeyframe();
            // Adds the current keyframe to the processed keyframes collection
            void AddToProcessedKeyframes();
            // Use deep feature vector to find potential loop candidate
            bool LoopKeyframeCandidate();
            // Process new keyframes with Loop Closure pipeline
            void LoopClosureLoop();


            Map::Ptr map_{nullptr};
            std::shared_ptr<Viewer> viewer_{nullptr};
            Camera::Ptr cam_left_{nullptr};
            Camera::Ptr cam_right_{nullptr};

            std::thread loopclosure_thread_;
            std::mutex database_mutex_;
            std::mutex list_mutex_;
            
            std::atomic<bool> loopclosure_running_;
            
            /* Deep Neural Network vision backbone for extracting 
             * feature vector from images */
            cv::dnn::Net network_;

            // Most recent keyframe in Loop Closure pipeline
            Frame::Ptr current_keyframe_{nullptr};
            // Previous most recent keyframe in Loop Closure pipeline
            Frame::Ptr last_keyframe_{nullptr};
            // Last keyframe that a loop was closed
            Frame::Ptr last_closed_keyframe_{nullptr};
            /* The candidate keyframe that is considered to create a loop with 
             * the current keyframe */
            Frame::Ptr candid_loop_keyframe_{nullptr};

            /* A list of keyframes that are waiting be processed
             * by loop closure pipeline */
            std::list<Frame::Ptr> waitlist_keyframes_;
            /* Keyframes that their deep feature vector representation
             * and keypoint features were calculated before and their
             * loop closure situation were checked */
            std::unordered_map<unsigned long, Frame::Ptr> processed_keyframes_;           
            

            // Hyperparameters

            /* If a loop was closed, ignore next n keyframe */
            int keyframes_to_ignore_after_loop_{6};
            float potential_loop_weak_threshold_{0.92};
            float potential_loop_strong_threshold_{0.95};
            int max_num_weak_threshold_{3};

    };

}

#endif