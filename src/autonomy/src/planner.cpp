// Minimal mission planner: loads a YAML task list into a matrix, publishes one
// absolute setpoint at a time on /planner/task, and advances when the
// controller acks the current task's seq on /controller/task_done.
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "snappy_interfaces/msg/task.hpp"

using std::placeholders::_1;

float total_x =0.0f;
float total_y =0.0f;
float total_z =0.0f;



class Planner : public rclcpp::Node
{
public:
    Planner() : Node("planner")
    {
        declare_parameter("task_file", std::string(""));
        std::string task_file = get_parameter("task_file").as_string();
        if (task_file.empty()) {
            throw std::runtime_error("task_file parameter is required");
        }

        YAML::Node root = YAML::LoadFile(task_file);
        YAML::Node yaml_tasks = root["tasks"];
        if (!yaml_tasks || !yaml_tasks.IsSequence() || yaml_tasks.size() == 0) {
            throw std::runtime_error("task_file has no 'tasks' sequence: " + task_file);
        }

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
        //RCLCPP_INFO(this->get_logger(), "Loaded %ld tasks from %s", tasks_.rows(), task_file.c_str());

        // Transient local so the controller still gets the current task if it
        // starts (or restarts) after the planner published it.
        task_publisher_ = this->create_publisher<snappy_interfaces::msg::Task>(
            "/planner/task", rclcpp::QoS(1).transient_local());

        done_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/controller/task_done", 10, std::bind(&Planner::done_callback, this, _1));

        publish_task(0);
    }

private:
    void publish_task(int seq)
    {
        auto msg = snappy_interfaces::msg::Task();
        msg.seq = seq;
        msg.x = tasks_(seq, 0) + total_x;
        msg.y = tasks_(seq, 1) + total_y;
        msg.z = tasks_(seq, 2) + total_z;

        msg.pitch = tasks_(seq, 3) * EIGEN_PI / 180.0;
        msg.roll = tasks_(seq, 4) * EIGEN_PI / 180.0;
        msg.yaw = tasks_(seq, 5) * EIGEN_PI / 180.0;

        current_seq_ = seq;
        task_publisher_->publish(msg);
        //RCLCPP_INFO(this->get_logger(), "Published task %d of %ld", seq, tasks_.rows());

        total_x += msg.x;
        total_y += msg.y;
        total_z += msg.z;
    }

    void done_callback(const std_msgs::msg::Int32 & msg)
    {
        if (mission_complete_ || msg.data != current_seq_) {
            return;
        }

        int next = current_seq_ + 1;
        if (next >= tasks_.rows()) {
            mission_complete_ = true;
            //RCLCPP_INFO(this->get_logger(), "mission complete");
            return;
        }
        publish_task(next);
    }

    Eigen::MatrixXd tasks_;
    int current_seq_ = 0;
    bool mission_complete_ = false;

    rclcpp::Publisher<snappy_interfaces::msg::Task>::SharedPtr task_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr done_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Planner>());
    rclcpp::shutdown();
    return 0;
}
