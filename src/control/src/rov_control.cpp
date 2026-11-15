#include "rov_control.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

RovControl::RovControl(rclcpp::Node & node, PID & depth_pid, ThrusterAllocator & allocator)
    : node_(node), depth_pid_(depth_pid), allocator_(allocator), config_(load_rov_config(node)),
      motor_output_enabled_(rov_parameter(node, "motor_output_enabled", false)), depth_(config_) {
    if (motor_output_enabled_ && !config_.depth_limits_configured())
        throw std::invalid_argument("Physical motor output requires explicit min_depth_m and max_depth_m limits");
    motor_ = node_.create_publisher<snappy_interfaces::msg::ThrusterCommand>(
        motor_output_enabled_ ? "/motor_cmd" : "/rov/disabled_motor_cmd", rclcpp::QoS(1).best_effort());
    allocated_ = node_.create_publisher<snappy_interfaces::msg::ThrusterCommand>("/rov/allocated_thrusters", 1);
    stop_state_ = node_.create_publisher<std_msgs::msg::Bool>(
        "/rov/emergency_stop", rclcpp::QoS(1).transient_local());
    publish_stop_state();
    commands_ = node_.create_subscription<snappy_interfaces::msg::RovCommand>("/rov/command", 1,
        [this](const snappy_interfaces::msg::RovCommand & msg) { receive(msg); });
    states_ = node_.create_subscription<snappy_interfaces::msg::Pose>("/state_estimator/state", 1,
        [this](const snappy_interfaces::msg::Pose & msg) {
            depth_.state(msg, rov_steady_seconds());
            if (depth_.healthy(rov_steady_seconds()))
                orientation_ = Eigen::Quaterniond(msg.orientation.w, msg.orientation.x,
                    msg.orientation.y, msg.orientation.z).normalized();
            else { needs_release_ = true; zero(); }
        });
    pressure_ = node_.create_subscription<std_msgs::msg::Float32>("/depth_data", 1,
        [this](const std_msgs::msg::Float32 & msg) {
            depth_.pressure(msg.data, rov_steady_seconds());
            if (!depth_.healthy(rov_steady_seconds())) { needs_release_ = true; zero(); }
        });
    reset_ = node_.create_service<std_srvs::srv::Trigger>("/rov/reset_emergency_stop",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
            if (killed_ || !command_fresh() || command_.emergency_stop || command_.deadman_pressed ||
                !depth_.healthy(rov_steady_seconds())) {
                response->success = false;
                response->message = "Reset requires fresh controller/depth data, L1 and Circle released, and an unexpired kill timer";
                return;
            }
            stop_latched_ = false;
            needs_release_ = true;
            command_valid_ = false;  // Require a new request after reset.
            zero();
            publish_stop_state();
            response->success = true;
            response->message = "Emergency stop reset; release then press L1 to resume";
        });
    RCLCPP_INFO(node_.get_logger(), "ROV motor output %s; allocated thrust preview: /rov/allocated_thrusters",
        motor_output_enabled_ ? "ENABLED" : "DISABLED (no /motor_cmd publisher)");
}

void RovControl::publish_stop_state() {
    std_msgs::msg::Bool msg;
    msg.data = stop_latched_ || killed_;
    stop_state_->publish(msg);
}

void RovControl::zero() {
    pid_ready_ = false;
    snappy_interfaces::msg::ThrusterCommand msg;
    msg.header.stamp = node_.now();
    msg.thruster_mask = 255;
    msg.thrust_pct.fill(0);
    motor_->publish(msg);
    allocated_->publish(msg);
}

void RovControl::kill() {
    killed_ = true;
    needs_release_ = true;
    zero();
    publish_stop_state();
}

bool RovControl::command_fresh() const {
    const double age = (node_.now() - rclcpp::Time(command_.header.stamp)).seconds();
    return command_valid_ && std::isfinite(age) && age >= -0.1 && age < config_.command_timeout &&
        rov_steady_seconds() - last_command_ < config_.command_timeout;
}

void RovControl::receive(const snappy_interfaces::msg::RovCommand & msg) {
    // Stop wins even over malformed/stale command fields.
    if (msg.emergency_stop) {
        stop_latched_ = true;
        needs_release_ = true;
        zero();
        publish_stop_state();
    }
    const double age = (node_.now() - rclcpp::Time(msg.header.stamp)).seconds();
    const bool valid = std::isfinite(msg.surge) && std::isfinite(msg.sway) && std::isfinite(msg.yaw) &&
        std::abs(msg.surge) <= config_.max_horizontal && std::abs(msg.sway) <= config_.max_horizontal &&
        std::abs(msg.yaw) <= config_.max_yaw && std::isfinite(msg.depth_target) &&
        (!msg.depth_enabled || (config_.depth_limits_configured() &&
            msg.depth_target >= config_.min_depth && msg.depth_target <= config_.max_depth)) &&
        (!msg.manual_enabled || msg.deadman_pressed) &&
        (msg.manual_enabled || (msg.surge == 0.0 && msg.sway == 0.0 && msg.yaw == 0.0)) &&
        msg.header.frame_id == "base_link" && std::isfinite(age) && age >= -0.1 && age < config_.command_timeout;
    if (!valid) {
        command_valid_ = false;
        needs_release_ = true;
        zero();
        return;
    }
    // Check the gap here too: a packet may arrive just before the watchdog timer.
    if (!command_fresh()) needs_release_ = true;
    const bool released_manual = command_.manual_enabled && !msg.manual_enabled;
    command_ = msg;
    last_command_ = rov_steady_seconds();
    command_valid_ = true;
    if (!msg.deadman_pressed && !stop_latched_ && !killed_ && depth_.healthy(last_command_))
        needs_release_ = false;
    if (!msg.depth_enabled) zero();
    else if (released_manual) tick();
}

double RovControl::effort(double normalized, int row) const {
    // Normalize against the existing allocation matrix and asymmetric motor
    // limits, so an isolated full-scale axis corresponds to full motor effort.
    const auto matrix = allocator_.get_configuration();
    double capacity = 0.0;
    const double direction = normalized < 0.0 ? -1.0 : 1.0;
    for (int col = 0; col < matrix.cols(); ++col)
        capacity += std::abs(matrix(row, col)) * (direction * matrix(row, col) < 0.0 ? 4.0 : 5.0);
    return normalized * capacity;
}

void RovControl::tick() {
    publish_stop_state();
    if (stop_latched_ || killed_ || !command_fresh() || !command_.depth_enabled ||
        !depth_.healthy(rov_steady_seconds()) || node_.count_publishers("/rov/command") != 1) {
        needs_release_ = true;
        zero();
        return;
    }
    double requested_target = command_.depth_target;
    if (pid_ready_ && requested_target < target_ &&
        depth_.depth() <= config_.min_depth + config_.surface_tolerance)
        requested_target = target_;
    if (!pid_ready_ || requested_target != target_) {
        target_ = requested_target;
        depth_pid_.set_target(static_cast<float>(target_));
        depth_pid_.reset(static_cast<float>(depth_.depth()));
        pid_ready_ = true;
    }
    // Reuse the original depth PID and buoyancy compensation; transform world
    // depth force into body coordinates. There is no yaw or heading PID here.
    const double vertical = depth_pid_.update(static_cast<float>(depth_.depth())) + 5.0;
    const Eigen::Vector3d force = orientation_.inverse() * Eigen::Vector3d(0.0, 0.0, vertical);
    Eigen::VectorXd wrench(6);
    wrench << force.x(), force.y(), force.z(), 0.0, 0.0, 0.0;
    if (!needs_release_ && command_.manual_enabled) {
        wrench[0] += effort(command_.surge, 0);
        wrench[1] += effort(command_.sway, 1);
        wrench[5] = effort(command_.yaw, 5);
    }
    const auto allocation = allocator_.allocate(wrench);
    if (!allocation.allFinite()) { needs_release_ = true; zero(); return; }
    snappy_interfaces::msg::ThrusterCommand msg;
    msg.header.stamp = node_.now();
    msg.header.frame_id = "base_link";
    msg.thruster_mask = 255;
    const auto matrix = allocator_.get_configuration();
    for (size_t i = 0; i < msg.thrust_pct.size(); ++i) {
        const double speed = allocation[i] * (allocation[i] > 0.0 ? 20.0 : 25.0);
        // Also bound combined surge/sway/yaw requests at the horizontal motors.
        const double limit = std::abs(matrix(2, i)) < 1e-6 ?
            100.0 * std::max(config_.max_horizontal, config_.max_yaw) : 100.0;
        msg.thrust_pct[i] = static_cast<int8_t>(std::clamp(speed, -limit, limit));
    }
    allocated_->publish(msg);
    if (!motor_output_enabled_) msg.thrust_pct.fill(0);
    motor_->publish(msg);
}
