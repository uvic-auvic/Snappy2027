#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <filesystem>

using namespace std;
using namespace cv;

class ImageCapture : public rclcpp::Node
{
public:
    ImageCapture() : Node("image_capture"){

        cout << "Test1" << endl;

        // Inititalize OpenCV windows
        // namedWindow("RealSense Color Stream", WINDOW_AUTOSIZE);
        //namedWindow("RealSense Depth Stream", WINDOW_AUTOSIZE);
        //namedWindow("RealSense Depth Colorized Stream", WINDOW_AUTOSIZE);
        //namedWindow("Sobel Edge Detected Stream", WINDOW_AUTOSIZE);

        // Inititalize subscribers
        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/d455/color/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&ImageCapture::color_callback, this, std::placeholders::_1));
    }

private:
    // Build timestamped filename: ../<type>/YYYYMMDD_HHMMSS_mmm.png
    static string name_image(const string &type, const string &ext = "png"){
    using namespace std::chrono;
    auto now = system_clock::now();
    time_t tt = system_clock::to_time_t(now);

    // Milliseconds for extra uniqueness
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    // Thread-safe localtime
    tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif

    // Ensure output folder exists
    filesystem::create_directories("../" + type);

    ostringstream oss;
    oss << "../" << type << "/"
        << put_time(&tm, "%Y%m%d_%H%M%S")
        << '_' << setw(3) << setfill('0') << ms.count()
        << '.' << ext;
    return oss.str();
    }

    // Compute Sobel derivative-based edges from 8-bit grayscale image
    // Returns a CV_8U mono image
    static Mat compute_sobel_edges(const Mat &gray, int ksize = 3, double scale = 1.0, double delta = 0.0){
        if (gray.empty()) return Mat();

        Mat src_gray;
        Mat edges;
        int depth = CV_16S;

        Mat grad_x, grad_y;
        Mat abs_grad_x, abs_grad_y;

        cout << "Test4" << std::endl;

        GaussianBlur(gray, gray, Size(3, 3), 0, 0, BORDER_DEFAULT);

        // cout << "Test5" << std::endl;

        // Convert to gray
        // cvtColor(gray, src_gray, COLOR_BGR2GRAY);

        cout << "Test6" << std::endl;

        // Gradient X
        Sobel(gray, grad_x, depth, 1, 0, ksize, scale, delta, BORDER_DEFAULT);
        convertScaleAbs(grad_x, abs_grad_x);

	cout << "Test7" << std::endl;

        // Gradient Y
        Sobel(gray, grad_y, depth, 0, 1, ksize, scale, delta, BORDER_DEFAULT);
        convertScaleAbs(grad_y, abs_grad_y);

        // Total Gradient (approximate)
        addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edges);
        return edges;
    }

    void color_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        // Convert image and display
        try {
            RCLCPP_INFO(this->get_logger(), "color_callback: received image");
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            imshow("RealSense Color Stream", cv_ptr->image);
            waitKey(1);

            // Save image with timestamped filename
            string filename = name_image("color");
            imwrite(filename, cv_ptr->image);
        }
        catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert depth image to a displayable 8-bit image
        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);

            Mat display; // 8-bit single-channel image for display/processing

            // Handle common depth encodings TODO:what is the resolution of depth images?
            if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                // Convert 16-bit depth to 8-bit for display, ignoring zeros (invalid)
                Mat depth16 = cv_ptr->image;
                Mat validMask = depth16 > 0;
                double minVal = 0.0, maxVal = 0.0;
                if (countNonZero(validMask) > 0) {
                    minMaxIdx(depth16, &minVal, &maxVal, nullptr, nullptr, validMask);
                }
                double scale = (maxVal > 0.0) ? 255.0 / maxVal : 1.0;
                depth16.convertTo(display, CV_8U, scale);
            }
            else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                // Convert 32-bit float depth (meters) to 8-bit, ignoring zeros
                Mat depth32 = cv_ptr->image;
                Mat validMask = depth32 > 0.0f;
                double minVal = 0.0, maxVal = 0.0;
                if (countNonZero(validMask) > 0) {
                    minMaxIdx(depth32, &minVal, &maxVal, nullptr, nullptr, validMask);
                }
                double scale = (maxVal > 0.0) ? 255.0 / maxVal : 1.0;
                depth32.convertTo(display, CV_8U, scale);
            }
            else if (cv_ptr->image.type() == CV_8UC1) {
                // Already 8-bit mono
                display = cv_ptr->image;
            }
            else {
                // Fallback: convert color to grayscale
                cvtColor(cv_ptr->image, display, COLOR_BGR2GRAY);
            }

            // Show a colorized depth view with histogram equalization
            Mat depth_eq;
            equalizeHist(display, depth_eq);
            Mat depth_colorized;
            applyColorMap(depth_eq, depth_colorized, COLORMAP_JET);
            //imshow("RealSense Depth Colorized Stream", depth_colorized);

		cout << "Test3" << std::endl;
            // Publish Sobel edges as mono8 image
            Mat edges = compute_sobel_edges(display);
            cv_bridge::CvImage out_msg(msg->header, sensor_msgs::image_encodings::MONO8, edges);
            sensor_msgs::msg::Image::SharedPtr ros_img = out_msg.toImageMsg();

            waitKey(1);
        }
        catch (const cv_bridge::Exception &e) {
        //    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (depth): %s", e.what());
        }
    }

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageCapture>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
