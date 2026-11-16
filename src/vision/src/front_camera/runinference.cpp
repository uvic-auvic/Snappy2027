#include "front_camera_vision.hpp"

std::vector<Detection> FrontCameraVision::run_inference(const cv::Mat & image, float & inference_ms)
    {
        auto t0 = std::chrono::steady_clock::now();

        const int orig_h = image.rows;
        const int orig_w = image.cols;

        // Letterbox resize
        cv::Mat resized;
        float scale;
        int pad_x, pad_y;
        letterbox(image, resized, scale, pad_x, pad_y);

        // BGR -> RGB, scale to [0,1], HWC -> CHW (NCHW blob) in one vectorized
        // call. `resized` is already letterboxed to input size, so the Size arg
        // is a no-op resize; swapRB handles BGR->RGB. `blob` must stay alive
        // until the stream sync below (cudaMemcpyAsync only queues the copy).
        cv::Mat blob = cv::dnn::blobFromImage(
            resized, 1.0 / 255.0, cv::Size(input_w_, input_h_),
            cv::Scalar(), /*swapRB=*/true, /*crop=*/false);

        cudaMemcpyAsync(device_input_, blob.ptr<float>(),
                        blob.total() * sizeof(float),
                        cudaMemcpyHostToDevice, stream_);

        context_->enqueueV3(stream_);

        cudaMemcpyAsync(host_output0_, device_output0_,
                        output0_size_ * sizeof(float),
                        cudaMemcpyDeviceToHost, stream_);
        cudaMemcpyAsync(host_output1_, device_output1_,
                        output1_size_ * sizeof(float),
                        cudaMemcpyDeviceToHost, stream_);

        cudaStreamSynchronize(stream_);

        auto detections = parse_segmentation_output(
            host_output0_, host_output1_, orig_w, orig_h, scale, pad_x, pad_y);

        auto t1 = std::chrono::steady_clock::now();
        inference_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                             "Inference: %.2f ms (%.1f FPS), %zu detections",
                             inference_ms, 1000.0f / inference_ms, detections.size());

        return detections;
    }