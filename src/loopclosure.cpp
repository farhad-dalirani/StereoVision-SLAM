#include<StereoVisionSLAM/loopclosure.h>
#include<StereoVisionSLAM/visual_odometry.h>
#include "StereoVisionSLAM/config.h"

namespace slam
{

    void LoopClosure::SetCameras(Camera::Ptr left, Camera::Ptr right) 
    {
        cam_left_ = left;
        cam_right_ = right;
    }

    void LoopClosure::SetMap(Map::Ptr map) 
    { 
        map_ = map; 
    }

    void LoopClosure::SetViewer(std::shared_ptr<Viewer> viewer)
    {
        viewer_ = viewer;
    }

    LoopClosure::LoopClosure()
    {
        // Set hyperparameters
        keyframes_to_ignore_after_loop_ = Config::Get<int>("keyframes_to_ignore_after_loop");

        // Initialize Deep Neural Network for feature extraction from images
        InitialFeatureExtractorNetwork();

        // Initialize backend optimization in a seprate thread
        loopclosure_running_.store(true);
        loopclosure_thread_ = std::thread(std::bind(&LoopClosure::LoopClosureLoop, this));
    }

    void LoopClosure::InitialFeatureExtractorNetwork()
    {
        /* 
        Create a vision feature extractor using MobileNet V2 pre-trained neural network ONNX file
        with OpenCV DNN. This deep neural network will be utilized to extract feature
        vectors from images, which will help in identifying potential loop closure candidates.
        */

        std::string modelPath = "./dnn_weights/mobilenet_v2.onnx";
        network_ = cv::dnn::readNetFromONNX(modelPath);

        // Check if the model was loaded successfully
        if (network_.empty()) 
        {
            throw SLAMException("Failed to load the Mobilenet V2 ONNX file!");
        }

    }

    void LoopClosure::ExtractFeatureVec(Frame::Ptr frame)
    {
        /* Extract feature vector representation of left image of
         * a frame with the backbone neural network */

        // Preprocess the image: resize, convert to blob, etc.
        cv::Mat blob;
        cv::dnn::blobFromImage(frame->left_img_, blob, 1.0/255.0, cv::Size(224, 224), 
                                cv::Scalar(0.485, 0.456, 0.406), true, false);

        // Set the input blob to the network
        network_.setInput(blob);

        // Perform the forward pass to get the feature vector
        std::string featureLayerName = "/GlobalAveragePool_output_0"; 
        cv::Mat output = network_.forward(featureLayerName);

        // Copy data from cv::Mat to Eigen vector
        std::memcpy(frame->representation_vec_.data(), output.ptr<float>(), 1280 * sizeof(float));

        // Normalization
	    frame->representation_vec_ /= frame->representation_vec_.norm();

        // Set the image has vector representation
        frame->has_rep_vec_ = true;
    }

    float LoopClosure::SimilarityScore(const Eigen::Matrix<float, 1280, 1> &a, 
                                    const Eigen::Matrix<float, 1280, 1> &b)
    {
        /* Calculate similarity score of two images feature vectors that
         * are extracted with the deep neural network backbone */
        float score = a.transpose() * b;
        return score;
    }

    void LoopClosure::AddNewKeyFrame(Frame::Ptr new_keyframe)
    {
        /* Add a new keyframe to waiting queue to be
         * processed by loop closure pipeline */

        std::unique_lock<std::mutex> lck(list_mutex_);

        /* If recently no loop was detected, add the new 
         * keyframe to the waitlist. */  
        if ((last_closed_keyframe_ == nullptr) || 
             (new_keyframe->keyframe_id_ - last_closed_keyframe_->keyframe_id_ >
                 keyframes_to_ignore_after_loop_))
        {
            waitlist_keyframes_.push_back(new_keyframe);
        }
        
    }

    bool LoopClosure::IsKeyframeInWaitingList()
    {
        // Return True if there are keyframes waiting to be check for loop
        std::unique_lock<std::mutex> lck(list_mutex_);
        return not(waitlist_keyframes_.empty());
    }

    void LoopClosure::LoopClosureLoop()
    {
        /* Running in another thread, constantly check new keyframes,
         * if there is a loop detected, call loop closure pipeline to 
         * correct camera poses and landmarks locations */

        while(loopclosure_running_.load())
        {
            std::unique_lock<std::mutex> lock(database_mutex_);
            
            //

            
        }
    }

    void LoopClosure::Stop() 
    {
        // Close loop closure optimization
        loopclosure_running_.store(false);
        loopclosure_thread_.join();
    }

    

}