#ifndef LOOPCLOSURE_H
#define LOOPCLOSURE_H

#include "StereoVisionSLAM/common_include.h"
#include "StereoVisionSLAM/map.h"
#include "StereoVisionSLAM/camera.h"
#include "StereoVisionSLAM/viewer.h"
#include "StereoVisionSLAM/frontend.h"
#include "StereoVisionSLAM/backend.h"
#include <opencv2/dnn.hpp>

namespace slam
{
    class Frontend;
    
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
            void SetBackend(std::shared_ptr<Backend> backend);
            void SetFrontend(std::shared_ptr<Frontend> frontend);
            void SetCameras(Camera::Ptr left, Camera::Ptr right);
            // Add a new keyframe to a queue to be process by loop closure pipeline
            void AddNewKeyFrame(Frame::Ptr new_keyframe);
            
            /* A lock for controlling update of landmarks' postion and 
             * keyframes' pose after detecting a loop to avoid coflict 
             * with frontend */ 
            std::mutex loop_closure_upd_;
            
        private:
            // Create deep neural netwrok to map a image to a feature vector 
            void InitialFeatureExtractorNetwork();
            // Extract feature representation of image by the backbone
            void ExtractFeatureVec(Frame::Ptr frame);
            // Extract descriptor for keypoints features
            void ExtractKeypointsDescriptor(Frame::Ptr frame);
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
            /* Check if a good keypoint feature mathcing between keypoint
             * of current frame and candidate frame for loop exist */
            bool KeypointMatchWithLoopCandid();
            /* Use keypoints and landmarks from looped keyframes
             * to calculate the current keyframe's pose. */
            bool CalculatePose();
            // Resolve conflict of closed loop and frontend/backend
            void LocalFusion();
            /* After detecting loop, correct pose of keyframes and
             * position of landmarks accordingly */
            void LoopClosureUpdate();
            // Process new keyframes with Loop Closure pipeline
            void LoopClosurePipeline();
            

            Map::Ptr map_{nullptr};
            std::weak_ptr<Backend> backend_;
            std::weak_ptr<Frontend> frontend_;
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
            // ORB keypoint feature descriptor
            cv::Ptr<cv::ORB> orb_descriptor_;
            // Keypoint feature matcher
            cv::Ptr<cv::DescriptorMatcher> matcher_;

            // Most recent keyframe in Loop Closure pipeline
            Frame::Ptr current_keyframe_{nullptr};
            // Previous most recent keyframe in Loop Closure pipeline
            Frame::Ptr last_keyframe_{nullptr};
            // Last keyframe that a loop was closed
            Frame::Ptr last_closed_keyframe_{nullptr};
            /* The candidate keyframe that is considered to create a loop with 
             * the current keyframe */
            Frame::Ptr candid_loop_keyframe_{nullptr};
            /* Index of left image keypoint features in 
             * current keyframe and their correspondance in
             * loop candidate keyframe. 
             * (candid keyframe feature index, current keyframe feature index) */
            std::set<std::pair<size_t, size_t>> KeypointMatches_;
            /* Corrected pose of current keyframe in world coordinate 
             * obtained by using new information after detecting a loop */
            Sophus::SE3d current_frame_corrected_pose_;
            /* If pose of current frame and its new pose after
             * detectiong a loop so similar, there is no need to
             * correct pose of keyframes */ 
            bool need_correction_{true};
            /* A list of keyframes that are waiting be processed
             * by loop closure pipeline */
            std::list<Frame::Ptr> waitlist_keyframes_;
            /* Keyframes that their deep feature vector representation
             * and keypoint features were calculated before and their
             * loop closure situation were checked.
             * (index in keyframe candidate for loop, index in current keyframe) */
            std::unordered_map<unsigned long, Frame::Ptr> processed_keyframes_;           
            

            // Hyperparameters

            /* If a loop was closed, ignore next n keyframe */
            int keyframes_to_ignore_after_loop_{6};
            float potential_loop_weak_threshold_{0.92};
            float potential_loop_strong_threshold_{0.95};
            int max_num_weak_threshold_{3};
            /* Minimum number of corresponding keypoints in
             * current keyframe and a candidate keyframe for loop closure
             * to count the candidate as a successful loop */
            int min_num_acceptable_keypoint_match_{11};
    };

}

#endif