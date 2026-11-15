#ifndef SNAPPY_ROV_CONTROL_HPP
#define SNAPPY_ROV_CONTROL_HPP

#include "pid.hpp"
#include "thruster_allocator.hpp"
#include "rov_parameters.hpp"
#include "snappy_interfaces/msg/rov_command.hpp"
#include "snappy_interfaces/msg/thruster_command.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"

// Launched only by the rov_depth_controller executable. The autonomous
// controller retains its original subscriptions, PID path and kill deadline.
class RovControl {
public:
    using MotorPublisher = rclcpp::Publisher<snappy_interfaces::msg::ThrusterCommand>;
    RovControl(rclcpp::Node & node, PID & depth_pid, ThrusterAllocator & allocator);
    MotorPublisher::SharedPtr motor_publisher() const { return motor_; }
    void tick();
    void kill();

private:
    void receive(const snappy_interfaces::msg::RovCommand & msg);
    bool command_fresh() const;
    void publish_stop_state();
    void zero();
    double effort(double normalized, int row) const;

    rclcpp::Node & node_;
    PID & depth_pid_;
    ThrusterAllocator & allocator_;
    const RovConfig config_;
    const bool motor_output_enabled_;
    RovDepthHealth depth_;
    snappy_interfaces::msg::RovCommand command_;
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    bool command_valid_ = false;
    bool stop_latched_ = false;
    bool killed_ = false;
    bool needs_release_ = true;
    bool pid_ready_ = false;
    double last_command_ = 0.0;
    double target_ = 0.0;
    MotorPublisher::SharedPtr motor_;
    MotorPublisher::SharedPtr allocated_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_state_;
    rclcpp::Subscription<snappy_interfaces::msg::RovCommand>::SharedPtr commands_;
    rclcpp::Subscription<snappy_interfaces::msg::Pose>::SharedPtr states_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pressure_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
};

#endif
