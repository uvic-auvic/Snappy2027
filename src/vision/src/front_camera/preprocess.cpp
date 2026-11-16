#include "front_camera_vision.hpp"

void FrontCameraVision::letterbox(const cv::Mat & src, cv::Mat & dst,
                   float & scale, int & pad_x, int & pad_y)
    {
        scale = std::min(static_cast<float>(input_w_) / src.cols,
                         static_cast<float>(input_h_) / src.rows);
        int new_w = static_cast<int>(src.cols * scale);
        int new_h = static_cast<int>(src.rows * scale);
        pad_x = (input_w_ - new_w) / 2;
        pad_y = (input_h_ - new_h) / 2;

        cv::resize(src, dst, cv::Size(new_w, new_h));
        cv::copyMakeBorder(dst, dst, pad_y, pad_y, pad_x, pad_x,
                           cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    }