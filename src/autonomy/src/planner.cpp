// Minimal mission planner: loads a YAML task list into a matrix, publishes one
// accumulated absolute setpoint at a time on /planner/task, and advances when the
// controller acks the current task's seq on /controller/task_done.
#include <stdexcept>
#include <string>
#include <chrono>
#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "snappy_interfaces/msg/task.hpp"

using std::placeholders::_1;

class Planner : public rclcpp::Node
{
public:
    Planner() : Node("planner")
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        declare_parameter("task_file", std::string(""), descriptor);
        task_timeout_ = declare_parameter("task_timeout_s", 60.0, descriptor);
        if (!std::isfinite(task_timeout_) || task_timeout_ <= 0)
            throw std::invalid_argument("task_timeout_s must be finite and positive");
        std::string task_file = get_parameter("task_file").as_string();
        if (task_file.empty()) {
            throw std::runtime_error("task_file parameter is required");
        }

        YAML::Node root = YAML::LoadFile(task_file);
        YAML::Node yaml_tasks = root["tasks"];
        if (!yaml_tasks || !yaml_tasks.IsSequence() || yaml_tasks.size() == 0) {
            throw std::runtime_error("task_file has no 'tasks' sequence: " + task_file);
        }
        if (yaml_tasks.size() > static_cast<size_t>(std::numeric_limits<int32_t>::max()))
            throw std::invalid_argument("Too many mission tasks");

        // One row per task, columns [x, y, z, pitch, roll, yaw].
        // Angles stay in degrees here; converted to radians when published.
        tasks_ = Eigen::MatrixXd(yaml_tasks.size(), 6);
        for (size_t i = 0; i < yaml_tasks.size(); i++) {
            const YAML::Node & t = yaml_tasks[i];
            tasks_(i, 0) = t["x"].as<double>();
            tasks_(i, 1) = t["y"].as<double>();
            tasks_(i, 2) = t["z"].as<double>();
            tasks_(i, 3) = t["pitch"].as<double>();
            tasks_(i, 4) = t["roll"].as<double>();
            tasks_(i, 5) = t["yaw"].as<double>();
        }
        if (!tasks_.allFinite()) throw std::invalid_argument("Mission tasks must contain finite values");
        // Task positions are relative translations along fixed mission axes.
        // Precompute cumulative targets once, so retransmission never moves them.
        for (Eigen::Index i = 1; i < tasks_.rows(); ++i)
            tasks_.block<1,3>(i, 0) += tasks_.block<1,3>(i - 1, 0);
        if (!tasks_.allFinite()) throw std::invalid_argument("Accumulated mission target overflow");
        //RCLCPP_INFO(this->get_logger(), "Loaded %ld tasks from %s", tasks_.rows(), task_file.c_str());

        // Transient local so the controller still gets the current task if it
        // starts (or restarts) after the planner published it.
        task_publisher_ = this->create_publisher<snappy_interfaces::msg::Task>(
            "/planner/task", rclcpp::QoS(1).transient_local());

        done_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/controller/task_done", 10, std::bind(&Planner::done_callback, this, _1));

        abort_publisher_ = create_publisher<std_msgs::msg::String>("/planner/abort", rclcpp::QoS(1).transient_local());
        status_publisher_ = create_publisher<std_msgs::msg::String>("/planner/status", rclcpp::QoS(1).transient_local());
        status_subscription_ = create_subscription<std_msgs::msg::String>("/controller/status", rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::String& status) {
                if (status.data.rfind("failed:", 0) == 0) fail(status.data);
            });

        publish_task(0);
        retry_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
            if (mission_complete_ || failed_) return;
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - task_started_).count() >= task_timeout_) {
                fail("Task " + std::to_string(current_seq_) + " timed out waiting for completion");
                return;
            }
            task_publisher_->publish(current_task_);
        });
    }

private:
    void publish_task(int seq)
    {
        auto msg = snappy_interfaces::msg::Task();
        msg.seq = seq;
        msg.x = tasks_(seq, 0);
        msg.y = tasks_(seq, 1);
        msg.z = tasks_(seq, 2);

        msg.pitch = tasks_(seq, 3) * EIGEN_PI / 180.0;
        msg.roll = tasks_(seq, 4) * EIGEN_PI / 180.0;
        msg.yaw = tasks_(seq, 5) * EIGEN_PI / 180.0;

        current_seq_ = seq;
        current_task_ = msg;
        task_started_ = std::chrono::steady_clock::now();
        task_publisher_->publish(msg);
        //RCLCPP_INFO(this->get_logger(), "Published task %d of %ld", seq, tasks_.rows());

        publish_status("tracking:" + std::to_string(seq));
        RCLCPP_INFO(get_logger(), "Task %d/%ld: target [%.3f, %.3f, %.3f] m",
                    seq + 1, static_cast<long>(tasks_.rows()), msg.x, msg.y, msg.z);
    }

    void done_callback(const std_msgs::msg::Int32 & msg)
    {
        if (mission_complete_ || failed_ || msg.data != current_seq_) {
            return;
        }

        int next = current_seq_ + 1;
        if (next >= tasks_.rows()) {
            mission_complete_ = true;
            publish_status("completed:holding_final_target");
            //RCLCPP_INFO(this->get_logger(), "mission complete");
            return;
        }
        publish_task(next);
    }

    void publish_status(const std::string& text) {
        std_msgs::msg::String message;
        message.data = text;
        status_publisher_->publish(message);
        RCLCPP_INFO(get_logger(), "%s", text.c_str());
    }

    void fail(const std::string& reason) {
        if (failed_) return;
        failed_ = true;
        std_msgs::msg::String message;
        message.data = reason;
        abort_publisher_->publish(message);
        publish_status("failed:" + reason);
        RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    }

    Eigen::MatrixXd tasks_;
    int current_seq_ = 0;
    bool mission_complete_ = false;
    bool failed_ = false;
    double task_timeout_;
    std::chrono::steady_clock::time_point task_started_;
    snappy_interfaces::msg::Task current_task_;
    rclcpp::TimerBase::SharedPtr retry_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr abort_publisher_, status_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;

    rclcpp::Publisher<snappy_interfaces::msg::Task>::SharedPtr task_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr done_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<Planner>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("planner"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
