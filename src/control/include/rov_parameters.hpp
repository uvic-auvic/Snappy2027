#ifndef SNAPPY_ROV_PARAMETERS_HPP
#define SNAPPY_ROV_PARAMETERS_HPP

#include "rclcpp/rclcpp.hpp"
#include "snappy_interfaces/msg/pose.hpp"
#include "rov_input.hpp"

#include <cmath>

template<class T>
T rov_parameter(rclcpp::Node & node, const std::string & name, T value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;  // Mapping and safety configuration require a restart.
    return node.declare_parameter<T>(name, value, descriptor);
}

inline RovConfig load_rov_config(rclcpp::Node & node) {
    RovConfig c;
    c.forward_axis = rov_parameter(node, "forward_axis", c.forward_axis);
    c.reverse_axis = rov_parameter(node, "reverse_axis", c.reverse_axis);
    c.strafe_axis = rov_parameter(node, "strafe_axis", c.strafe_axis);
    c.yaw_axis = rov_parameter(node, "yaw_axis", c.yaw_axis);
    c.dpad_axis = rov_parameter(node, "dpad_axis", c.dpad_axis);
    c.dpad_up_button = rov_parameter(node, "dpad_up_button", c.dpad_up_button);
    c.dpad_down_button = rov_parameter(node, "dpad_down_button", c.dpad_down_button);
    c.deadman_button = rov_parameter(node, "deadman_button", c.deadman_button);
    c.stop_button = rov_parameter(node, "stop_button", c.stop_button);
    c.trigger_released = rov_parameter(node, "trigger_released", c.trigger_released);
    c.trigger_pressed = rov_parameter(node, "trigger_pressed", c.trigger_pressed);
    c.invert_strafe = rov_parameter(node, "invert_strafe", c.invert_strafe);
    c.invert_yaw = rov_parameter(node, "invert_yaw", c.invert_yaw);
    c.invert_dpad = rov_parameter(node, "invert_dpad", c.invert_dpad);
    c.deadzone = rov_parameter(node, "deadzone", c.deadzone);
    c.max_horizontal = rov_parameter(node, "max_horizontal_thrust", c.max_horizontal);
    c.max_yaw = rov_parameter(node, "max_yaw_thrust", c.max_yaw);
    c.depth_increment = rov_parameter(node, "depth_increment_m", c.depth_increment);
    c.min_depth = rov_parameter(node, "min_depth_m", c.min_depth);
    c.max_depth = rov_parameter(node, "max_depth_m", c.max_depth);
    c.surface_tolerance = rov_parameter(node, "surface_tolerance_m", c.surface_tolerance);
    c.joy_timeout = rov_parameter(node, "controller_timeout_s", c.joy_timeout);
    c.depth_timeout = rov_parameter(node, "depth_timeout_s", c.depth_timeout);
    c.command_timeout = rov_parameter(node, "command_timeout_s", c.command_timeout);
    c.validate();
    return c;
}

inline double rov_steady_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Pose has no header, so monitor the pressure source AND estimator receptions.
// Fresh estimator predictions alone cannot establish pressure sensor health.
class RovDepthHealth {
public:
    explicit RovDepthHealth(const RovConfig & config) : config_(config) {}
    void pressure(double value, double time) {
        const bool recovering = !pressure_valid_ || time - pressure_time_ >= config_.depth_timeout;
        pressure_valid_ = std::isfinite(value) && value >= config_.min_depth &&
            config_.depth_limits_configured() && value <= config_.max_depth;
        pressure_time_ = time;
        // After pressure recovery, wait for an estimator sample that can reflect
        // the restored measurement before capturing a new depth target.
        if (recovering) state_valid_ = false;
    }
    void state(const snappy_interfaces::msg::Pose & pose, double time) {
        const auto & q = pose.orientation;
        const double norm = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
        state_valid_ = config_.depth_limits_configured() && std::isfinite(pose.position.z) &&
            pose.position.z >= config_.min_depth && pose.position.z <= config_.max_depth &&
            std::isfinite(norm) && std::abs(norm - 1.0) < 0.1;
        state_time_ = time;
        depth_ = pose.position.z;
    }
    bool healthy(double time) const {
        return pressure_valid_ && state_valid_ && time - pressure_time_ < config_.depth_timeout &&
            time - state_time_ < config_.depth_timeout;
    }
    double depth() const { return depth_; }
private:
    RovConfig config_;
    bool pressure_valid_ = false;
    bool state_valid_ = false;
    double pressure_time_ = 0.0;
    double state_time_ = 0.0;
    double depth_ = 0.0;
};

#endif
