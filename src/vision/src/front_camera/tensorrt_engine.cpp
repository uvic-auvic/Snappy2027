#include "front_camera_vision.hpp"

void FrontCameraVision::init_tensorrt()
    {
        std::ifstream file(engine_path_, std::ios::binary);
        if (!file.good()) {
            throw std::runtime_error("Failed to open TensorRT engine: " + engine_path_);
        }

        file.seekg(0, std::ios::end);
        size_t size = static_cast<size_t>(file.tellg());
        file.seekg(0, std::ios::beg);

        std::vector<char> engine_data(size);
        file.read(engine_data.data(), static_cast<std::streamsize>(size));
        file.close();

        // Ultralytics `yolo export format=engine` prepends a metadata blob:
        //   [int32 little-endian length][metadata JSON]["...serialized TRT plan"]
        // Skip it so deserialize sees the plan's magic tag. Plain trtexec engines
        // have no header (length byte won't look like a JSON-prefixed blob).
        size_t offset = 0;
        if (size > 4) {
            int32_t meta_len;
            std::memcpy(&meta_len, engine_data.data(), sizeof(meta_len));
            if (meta_len > 0 && static_cast<size_t>(meta_len) < size - 4 &&
                engine_data[4] == '{') {
                offset = 4 + static_cast<size_t>(meta_len);
                RCLCPP_INFO(get_logger(),
                            "Stripping %d-byte Ultralytics engine metadata header",
                            meta_len);
            }
        }

        runtime_ = nvinfer1::createInferRuntime(trt_logger_);
        if (!runtime_) throw std::runtime_error("Failed to create TRT runtime");

        engine_ = runtime_->deserializeCudaEngine(engine_data.data() + offset, size - offset);
        if (!engine_) throw std::runtime_error("Failed to deserialize TRT engine");

        context_ = engine_->createExecutionContext();
        if (!context_) throw std::runtime_error("Failed to create TRT execution context");

        // YOLOv26 seg has 3 I/O tensors: input, output0 (boxes+coeffs), output1 (proto)
        input_name_ = engine_->getIOTensorName(0);
        output0_name_ = engine_->getIOTensorName(1);
        output1_name_ = engine_->getIOTensorName(2);

        auto input_dims = engine_->getTensorShape(input_name_.c_str());
        input_h_ = input_dims.d[2];
        input_w_ = input_dims.d[3];

        auto out0_dims = engine_->getTensorShape(output0_name_.c_str());
        output0_size_ = 1;
        for (int i = 0; i < out0_dims.nbDims; ++i) output0_size_ *= out0_dims.d[i];
        // YOLO26 NMS-free: out0 = [1, num_det, attrs] e.g. (1, 300, 38)
        num_detections_ = out0_dims.d[1];
        det_attrs_      = out0_dims.d[2];

        auto out1_dims = engine_->getTensorShape(output1_name_.c_str());
        output1_size_ = 1;
        for (int i = 0; i < out1_dims.nbDims; ++i) output1_size_ *= out1_dims.d[i];
        mask_h_ = out1_dims.d[2];
        mask_w_ = out1_dims.d[3];

        // Allocate device buffers
        cudaStreamCreate(&stream_);
        cudaMalloc(&device_input_,   3 * input_h_ * input_w_ * sizeof(float));
        cudaMalloc(&device_output0_, output0_size_ * sizeof(float));
        cudaMalloc(&device_output1_, output1_size_ * sizeof(float));

        // Pinned host buffers for async D2H
        cudaHostAlloc(&host_output0_, output0_size_ * sizeof(float), cudaHostAllocDefault);
        cudaHostAlloc(&host_output1_, output1_size_ * sizeof(float), cudaHostAllocDefault);

        // Bind tensor addresses once
        context_->setTensorAddress(input_name_.c_str(),   device_input_);
        context_->setTensorAddress(output0_name_.c_str(), device_output0_);
        context_->setTensorAddress(output1_name_.c_str(), device_output1_);

        RCLCPP_INFO(get_logger(), "Engine loaded: %dx%d input | out0=%zu | out1=%zu",
                     input_w_, input_h_, output0_size_, output1_size_);
    }