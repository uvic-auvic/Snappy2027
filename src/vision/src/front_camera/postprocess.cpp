#include "front_camera_vision"

std::vector<Detection> FrontCameraVision::parse_segmentation_output(
        const float * output0, const float * output1,
        int orig_w, int orig_h, float scale, int pad_x, int pad_y)
    {
        // YOLO26 is NMS-free / end-to-end: output0 layout is
        //   [1, num_det, 4 + 1 + 1 + 32]  e.g. (1, 300, 38)
        // per detection: [x1, y1, x2, y2, conf, class_id, m0..m31]
        //   - boxes are xyxy in letterboxed INPUT pixel space (0..input_w/h)
        //   - detections are already filtered + sorted by conf (no manual NMS)
        // output1 (proto masks) layout: [1, 32, mask_h_, mask_w_]
        const int num_det = num_detections_;          // out0_dims.d[1], e.g. 300
        const int attrs   = det_attrs_;               // out0_dims.d[2], e.g. 38
        const int num_mask_coeffs = attrs - 6;        // 38 - 6 = 32
        constexpr float mask_threshold = 0.5f;

        std::vector<Detection> detections;

        for (int i = 0; i < num_det; ++i) {
            const int offset = i * attrs;
            const float conf = output0[offset + 4];

            // Sorted by confidence desc: once below threshold, the rest are too.
            if (conf < static_cast<float>(conf_threshold_)) break;

            const int class_id = static_cast<int>(std::lround(output0[offset + 5]));

            Detection det;
            det.x1 = std::clamp((output0[offset + 0] - pad_x) / scale, 0.0f, static_cast<float>(orig_w));
            det.y1 = std::clamp((output0[offset + 1] - pad_y) / scale, 0.0f, static_cast<float>(orig_h));
            det.x2 = std::clamp((output0[offset + 2] - pad_x) / scale, 0.0f, static_cast<float>(orig_w));
            det.y2 = std::clamp((output0[offset + 3] - pad_y) / scale, 0.0f, static_cast<float>(orig_h));
            det.confidence = conf;
            det.class_id = class_id;

            std::vector<float> coeffs(num_mask_coeffs);
            for (int k = 0; k < num_mask_coeffs; ++k) {
                coeffs[k] = output0[offset + 6 + k];
            }
            det.mask_polygon = decode_mask_polygon(coeffs, output1, det,
                                                    orig_w, orig_h, pad_x, pad_y,
                                                    mask_threshold);

            detections.push_back(std::move(det));
        }

        // Debug: dump the top detection's raw values so the box format can be
        // verified (xyxy vs xywh) against ground truth.
        if (num_det > 0) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "raw det0: [%.1f %.1f %.1f %.1f] conf=%.3f cls=%.1f | kept=%zu",
                output0[0], output0[1], output0[2], output0[3],
                output0[4], output0[5], detections.size());
        }

        return detections;
    }

    // ---- Mask decoding: proto matmul -> sigmoid -> threshold -> contour ----

std::vector<cv::Point2f> FrontCameraVision::decode_mask_polygon(
        const std::vector<float> & mask_coeffs,
        const float * proto,
        const Detection & det,
        int orig_w, int orig_h,
        int pad_x, int pad_y,
        float mask_threshold)
    {
        const int num_mask_coeffs = static_cast<int>(mask_coeffs.size());

        // coeffs . proto -> low-res mask, then sigmoid.
        // proto is [num_mask_coeffs, mask_h_*mask_w_]; coeffs is [1, num_mask_coeffs].
        // A single GEMM (BLAS) replaces the per-pixel triple loop -- the dominant
        // CPU cost per detection.
        const cv::Mat proto_mat(num_mask_coeffs, mask_h_ * mask_w_, CV_32F,
                                const_cast<float *>(proto));
        const cv::Mat coeffs_mat(1, num_mask_coeffs, CV_32F,
                                 const_cast<float *>(mask_coeffs.data()));
        cv::Mat mask_low = coeffs_mat * proto_mat;        // [1, mask_h_*mask_w_]
        mask_low = mask_low.reshape(1, mask_h_);          // [mask_h_, mask_w_]

        // Sigmoid in place: 1 / (1 + exp(-x))
        cv::exp(-mask_low, mask_low);
        cv::add(mask_low, 1.0, mask_low);
        cv::divide(1.0, mask_low, mask_low);

        // Upscale proto mask to letterboxed input size
        cv::Mat mask_input;
        cv::resize(mask_low, mask_input, cv::Size(input_w_, input_h_), 0, 0, cv::INTER_LINEAR);

        // Crop out the letterbox padding, leaving only the real-image region
        cv::Rect crop_roi(pad_x, pad_y, input_w_ - 2 * pad_x, input_h_ - 2 * pad_y);
        crop_roi &= cv::Rect(0, 0, mask_input.cols, mask_input.rows);
        if (crop_roi.width <= 0 || crop_roi.height <= 0) {
            return {};
        }
        cv::Mat mask_cropped = mask_input(crop_roi);

        // Resize to original image dimensions
        cv::Mat mask_full;
        cv::resize(mask_cropped, mask_full, cv::Size(orig_w, orig_h), 0, 0, cv::INTER_LINEAR);

        // Threshold to binary
        cv::Mat mask_bin;
        cv::threshold(mask_full, mask_bin, mask_threshold, 255, cv::THRESH_BINARY);
        mask_bin.convertTo(mask_bin, CV_8U);

        // Gate by the detection's own box (proto masks are class-agnostic;
        // the box tells us which instance this mask belongs to)
        cv::Rect box(static_cast<int>(det.x1), static_cast<int>(det.y1),
                    static_cast<int>(det.x2 - det.x1), static_cast<int>(det.y2 - det.y1));
        box &= cv::Rect(0, 0, orig_w, orig_h);

        cv::Mat box_mask = cv::Mat::zeros(mask_bin.size(), CV_8U);
        if (box.width > 0 && box.height > 0) {
            mask_bin(box).copyTo(box_mask(box));
        }

        // Extract the largest external contour as the polygon
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(box_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty()) {
            return {};
        }

        auto largest = std::max_element(contours.begin(), contours.end(),
            [](const auto & a, const auto & b) { return a.size() < b.size(); });

        std::vector<cv::Point2f> polygon;
        polygon.reserve(largest->size());
        for (const auto & pt : *largest) {
            polygon.emplace_back(static_cast<float>(pt.x), static_cast<float>(pt.y));
        }
        return polygon;
    }

/*
USED??
static float FrontCameraVision::compute_iou(const Anchor & a, const Anchor & b)
    {
        float x1 = std::max(a.x - a.w / 2.0f, b.x - b.w / 2.0f);
        float y1 = std::max(a.y - a.h / 2.0f, b.y - b.h / 2.0f);
        float x2 = std::min(a.x + a.w / 2.0f, b.x + b.w / 2.0f);
        float y2 = std::min(a.y + a.h / 2.0f, b.y + b.h / 2.0f);

        float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
        float union_area = a.w * a.h + b.w * b.h - inter;
        return (union_area > 0.0f) ? (inter / union_area) : 0.0f;
    }
*/