// GPU preprocessing for the shared camera inference node.
//
// A single fused CUDA kernel performs, per output pixel:
//   letterbox resize (bilinear) + BGR->RGB + normalize to [0,1] + HWC->CHW
// writing directly into the TensorRT input buffer. This removes the entire CPU
// preprocessing chain (cv::resize / cvtColor / convertTo / split + memcpy) that
// previously ran on the Orin Nano's CPU on every frame.
#pragma once

#include <cuda_runtime.h>
#include <cstddef>
#include <cstdint>

// src: device pointer to the source image, 8-bit BGR, HWC, src_step bytes/row.
// dst: device pointer to one sample's slice of the engine input (float, CHW,
//      dst_w*dst_h*3 elements), RGB planar, normalized to [0,1].
// scale/pad_x/pad_y: letterbox mapping (dst = src*scale, centered with padding).
// Returns the launch error (cudaGetLastError); kernel completion is observed
// when the caller synchronizes the stream.
cudaError_t launchLetterboxPreprocess(
    const uint8_t* src, int src_w, int src_h, size_t src_step,
    float* dst, int dst_w, int dst_h,
    float scale, int pad_x, int pad_y,
    cudaStream_t stream);
