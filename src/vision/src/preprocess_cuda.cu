// Fused letterbox + BGR->RGB + normalize + HWC->CHW preprocessing kernel.
// See preprocess_cuda.hpp for the contract.

#include "snappy_computer_vision/preprocess_cuda.hpp"

namespace {

__device__ __forceinline__ float bilerp(float v00, float v01, float v10, float v11,
                                        float ax, float ay)
{
    const float top = v00 + (v01 - v00) * ax;
    const float bot = v10 + (v11 - v10) * ax;
    return top + (bot - top) * ay;
}

// 114/255 -- the standard YOLO letterbox fill colour, pre-normalized.
__device__ constexpr float kPadFill = 114.0f / 255.0f;

__global__ void letterboxKernel(const unsigned char* __restrict__ src,
                                int src_w, int src_h, int src_step,
                                float* __restrict__ dst, int dst_w, int dst_h,
                                float inv_scale, float pad_x, float pad_y)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= dst_w || y >= dst_h) {
        return;
    }

    const int plane = dst_w * dst_h;
    const int idx = y * dst_w + x;

    float r = kPadFill, g = kPadFill, b = kPadFill;

    // Map this destination pixel back into source image coordinates.
    const float fx = (static_cast<float>(x) - pad_x) * inv_scale;
    const float fy = (static_cast<float>(y) - pad_y) * inv_scale;

    if (fx >= 0.0f && fy >= 0.0f &&
        fx <= static_cast<float>(src_w - 1) && fy <= static_cast<float>(src_h - 1)) {
        const int x0 = static_cast<int>(fx);
        const int y0 = static_cast<int>(fy);
        const int x1 = min(x0 + 1, src_w - 1);
        const int y1 = min(y0 + 1, src_h - 1);
        const float ax = fx - static_cast<float>(x0);
        const float ay = fy - static_cast<float>(y0);

        const unsigned char* p00 = src + y0 * src_step + x0 * 3;
        const unsigned char* p01 = src + y0 * src_step + x1 * 3;
        const unsigned char* p10 = src + y1 * src_step + x0 * 3;
        const unsigned char* p11 = src + y1 * src_step + x1 * 3;

        // Source is BGR: channel 0=B, 1=G, 2=R.
        const float bb = bilerp(p00[0], p01[0], p10[0], p11[0], ax, ay);
        const float gg = bilerp(p00[1], p01[1], p10[1], p11[1], ax, ay);
        const float rr = bilerp(p00[2], p01[2], p10[2], p11[2], ax, ay);

        const float inv255 = 1.0f / 255.0f;
        r = rr * inv255;
        g = gg * inv255;
        b = bb * inv255;
    }

    // Destination is RGB planar (CHW): plane 0=R, 1=G, 2=B.
    dst[0 * plane + idx] = r;
    dst[1 * plane + idx] = g;
    dst[2 * plane + idx] = b;
}

}  // namespace

cudaError_t launchLetterboxPreprocess(
    const uint8_t* src, int src_w, int src_h, size_t src_step,
    float* dst, int dst_w, int dst_h,
    float scale, int pad_x, int pad_y,
    cudaStream_t stream)
{
    const dim3 block(16, 16);
    const dim3 grid((dst_w + block.x - 1) / block.x,
                    (dst_h + block.y - 1) / block.y);

    const float inv_scale = (scale > 0.0f) ? (1.0f / scale) : 1.0f;

    letterboxKernel<<<grid, block, 0, stream>>>(
        reinterpret_cast<const unsigned char*>(src), src_w, src_h,
        static_cast<int>(src_step), dst, dst_w, dst_h,
        inv_scale, static_cast<float>(pad_x), static_cast<float>(pad_y));

    return cudaGetLastError();
}
