#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "snappy_interfaces/msg/pose.hpp"
#include "snappy_interfaces/msg/thruster_command.hpp"

#include "thruster_allocator.hpp"

using namespace std::chrono_literals;

struct MissionStep {
  std::string command = "stop";
  double duration = 0.0;
  double speed = 30.0;
  double target_depth = 0.0;
  double hold_time = 0.0;
  double tolerance = 0.2;
  std::string label = "step";
};

class TimeBasedController : public rclcpp::Node {
 public:
  TimeBasedController()
      : Node("time_based_controller"),
        thruster_allocator_(build_thruster_configuration(), -4.0, 5.0) {
    declare_parameter("mission_file", std::string(""));
    declare_parameter("default_speed", 35.0);
    declare_parameter("state_topic", std::string("/state_estimator/state"));
    declare_parameter("command_rate_hz", 20.0);
    declare_parameter("kill_timeout_s", 30.0);

    default_speed_ = get_parameter("default_speed").as_double();
    state_topic_ = get_parameter("state_topic").as_string();
    command_rate_hz_ = get_parameter("command_rate_hz").as_double();

    motor_publisher_ = create_publisher<snappy_interfaces::msg::ThrusterCommand>(
        "/motor_cmd", rclcpp::QoS(10).best_effort());

    const double kill_timeout_s = get_parameter("kill_timeout_s").as_double();
    if (kill_timeout_s <= 0.0) {
      throw std::invalid_argument("kill_timeout_s must be greater than zero");
    }
    kill_deadline_ = std::chrono::steady_clock::now()
      + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(kill_timeout_s));
    RCLCPP_WARN(
      get_logger(),
      "Thruster kill timer armed: all thrusters will stop in %.1f seconds",
      kill_timeout_s);

    status_publisher_ = create_publisher<std_msgs::msg::String>(
        "/time_based_controller/status", 10);

    state_subscription_ = create_subscription<snappy_interfaces::msg::Pose>(
        state_topic_, 10,
        std::bind(&TimeBasedController::state_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / command_rate_hz_),
                               std::bind(&TimeBasedController::timer_callback, this));

    std::string mission_file = get_parameter("mission_file").as_string();
    if (!mission_file.empty()) {
      load_mission_sequence(mission_file);
    }
    if (mission_steps_.empty()) {
      RCLCPP_WARN(get_logger(), "No mission file provided or sequence is empty. Using default stop-only mission.");
      mission_steps_.push_back({"stop", 0.0, 0.0, 0.0, 0.0, 0.0, "idle"});
    }

    publish_status("Mission starting");
  }

 private:
  static Eigen::MatrixXd build_thruster_configuration() {
    Eigen::MatrixXd config(6, 8);
    config << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
        0.1301, 0.1653, 0.0, 0.1653, -0.1301, -0.1648, 0.0, -0.1648,
        0.0, 0.3024, -0.0158, -0.2977, 0.0, -0.2977, -0.0159, 0.3024,
        -0.3041, 0.0, -0.2739, 0.0, -0.3121, 0.0, 0.2734, 0.0;
    return config;
  }

  void load_mission_sequence(const std::string & mission_file) {
    YAML::Node root;
    try {
      root = YAML::LoadFile(mission_file);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to load mission file '%s': %s", mission_file.c_str(), ex.what());
      return;
    }

    const YAML::Node steps = root["movement_sequence"];
    if (!steps || !steps.IsSequence()) {
      RCLCPP_ERROR(get_logger(), "Mission file '%s' does not contain a valid 'movement_sequence' list.", mission_file.c_str());
      return;
    }

    for (std::size_t i = 0; i < steps.size(); ++i) {
      const YAML::Node step_node = steps[i];
      MissionStep step;
      step.command = step_node["command"].as<std::string>("stop");
      step.duration = step_node["duration"].as<double>(0.0);
      step.speed = step_node["speed"].as<double>(default_speed_);
      step.target_depth = step_node["target_depth"].as<double>(0.0);
      step.hold_time = step_node["hold_time"].as<double>(0.0);
      step.tolerance = step_node["tolerance"].as<double>(0.2);
      step.label = step_node["label"].as<std::string>(step.command + "_" + std::to_string(i));
      mission_steps_.push_back(step);
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu mission steps from %s", mission_steps_.size(), mission_file.c_str());
  }

  void state_callback(const snappy_interfaces::msg::Pose::SharedPtr msg) {
    current_position_ = *msg;
    current_depth_ = msg->position.z;
    current_yaw_ = std::atan2(2.0 * (msg->orientation.w * msg->orientation.z +
                                  msg->orientation.x * msg->orientation.y),
                             1.0 - 2.0 * (msg->orientation.y * msg->orientation.y +
                                          msg->orientation.z * msg->orientation.z));
  }

  void timer_callback() {
    const rclcpp::Time now = this->now();

    if (kill_latched_ || std::chrono::steady_clock::now() >= kill_deadline_) {
      if (!kill_latched_) {
        kill_latched_ = true;
        RCLCPP_ERROR(
            get_logger(),
            "Thruster kill timer expired; all motor commands are now latched at zero");
      }
      publish_zero_thrust();
      return;
    }

    if (!mission_active_) {
      publish_zero_thrust();
      return;
    }

    if (mission_step_index_ >= mission_steps_.size()) {
      finish_mission("completed all movement steps");
      return;
    }

    if (!step_started_) {
      start_step(mission_steps_[mission_step_index_], now);
      return;
    }

    const MissionStep & step = mission_steps_[mission_step_index_];
    const double elapsed = (now - step_started_at_).seconds();

    if (step.command == "depth_target") {
      const double error = step.target_depth - current_depth_;
      if (std::abs(error) <= step.tolerance && elapsed >= 0.0) {
        const double hold_remaining = step.hold_time - depth_hold_elapsed_;
        if (hold_remaining > 0.0) {
          if (depth_hold_started_ == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
            depth_hold_started_ = now;
          }
          const double hold_elapsed = (now - depth_hold_started_).seconds();
          depth_hold_elapsed_ = hold_elapsed;
          publish_wrench(build_depth_hold_wrench(step.target_depth), "holding depth at target");
          if (hold_elapsed >= step.hold_time) {
            complete_current_step();
          }
          return;
        }
        complete_current_step();
        return;
      }

      if (elapsed >= step.duration && std::abs(error) > step.tolerance) {
        RCLCPP_WARN(get_logger(), "Depth target step timed out before reaching %.2f m (current %.2f m)", step.target_depth, current_depth_);
        complete_current_step();
        return;
      }

      publish_wrench(build_depth_target_wrench(step.target_depth, step.speed), "depth_target in progress");
      return;
    }

    if (step.command == "stop") {
      publish_zero_thrust();
      complete_current_step();
      return;
    }

    if (elapsed >= step.duration) {
      publish_zero_thrust();
      complete_current_step();
      return;
    }

    publish_wrench(build_body_wrench(step.command, step.speed), "timed movement in progress");
  }

  void start_step(const MissionStep & step, const rclcpp::Time & now) {
    step_started_ = true;
    step_started_at_ = now;
    depth_hold_elapsed_ = 0.0;
    depth_hold_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    if (step.command == "depth_target") {
      RCLCPP_INFO(get_logger(), "Starting depth target step %s: target depth %.2f m, hold %.1f s, max %.1f s", step.label.c_str(),
                  step.target_depth, step.hold_time, step.duration);
    } else {
      RCLCPP_INFO(get_logger(), "Starting step %s: command=%s duration=%.2f s speed=%.1f",
                  step.label.c_str(), step.command.c_str(), step.duration, step.speed);
    }
    publish_status("Step: " + step.label + " (" + step.command + ")");
  }

  void complete_current_step() {
    publish_zero_thrust();
    mission_step_index_ += 1;
    step_started_ = false;
    depth_hold_elapsed_ = 0.0;
    depth_hold_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    if (mission_step_index_ < mission_steps_.size()) {
      RCLCPP_INFO(get_logger(), "Completed step %zu/%zu", mission_step_index_, mission_steps_.size());
    }
  }

  Eigen::VectorXd build_body_wrench(const std::string & command, double speed) const {
    Eigen::VectorXd wrench(6);
    wrench.setZero();

    const double clamped_speed = std::clamp(speed, -100.0, 100.0);
    const double force_limit = std::clamp(std::abs(clamped_speed), 0.0, 100.0);
    const double body_force = force_limit / 20.0;

    if (command == "forward") {
      wrench[0] = body_force;
    } else if (command == "backward") {
      wrench[0] = -body_force;
    } else if (command == "left") {
      wrench[1] = -body_force;
    } else if (command == "right") {
      wrench[1] = body_force;
    } else if (command == "ascend") {
      wrench[2] = body_force;
    } else if (command == "descend") {
      wrench[2] = -body_force;
    }

    return wrench;
  }

  Eigen::VectorXd build_depth_target_wrench(double target_depth, double speed) const {
    Eigen::VectorXd wrench(6);
    wrench.setZero();

    const double error = target_depth - current_depth_;
    const double vertical_command = std::clamp((error * 3.0), -speed, speed);
    const double body_force = std::clamp(std::abs(vertical_command) / 20.0, 0.0, 5.0);
    wrench[2] = std::copysign(body_force, vertical_command);
    return wrench;
  }

  Eigen::VectorXd build_depth_hold_wrench(double target_depth) const {
    Eigen::VectorXd wrench(6);
    wrench.setZero();
    const double error = target_depth - current_depth_;
    const double vertical_command = std::clamp(error * 4.0, -30.0, 30.0);
    const double body_force = std::clamp(std::abs(vertical_command) / 20.0, 0.0, 5.0);
    wrench[2] = std::copysign(body_force, vertical_command);
    return wrench;
  }

  void publish_wrench(const Eigen::VectorXd & wrench, const std::string & reason) {
    if (!mission_active_) {
      publish_zero_thrust();
      return;
    }

    const auto allocation = allocate_thrusters(wrench);
    snappy_interfaces::msg::ThrusterCommand msg;
    msg.thruster_mask = 255;
    for (std::size_t i = 0; i < msg.thrust_pct.size(); ++i) {
      msg.thrust_pct[i] = allocation[i];
    }
    motor_publisher_->publish(msg);
    if (!reason.empty()) {
      publish_status(reason);
    }
  }

  void publish_zero_thrust() {
    snappy_interfaces::msg::ThrusterCommand msg;
    msg.header.stamp = now();
    msg.thruster_mask = 255;
    for (auto & value : msg.thrust_pct) {
      value = 0;
    }
    motor_publisher_->publish(msg);
  }

  std::vector<int8_t> allocate_thrusters(const Eigen::VectorXd & wrench) const {
    const Eigen::VectorXd allocation = thruster_allocator_.allocate(wrench);
    std::vector<int8_t> output;
    output.reserve(static_cast<std::size_t>(allocation.size()));
    for (int i = 0; i < allocation.size(); ++i) {
      const double force = allocation[i];
      output.push_back(thrust_to_speed(force));
    }
    return output;
  }

  int8_t thrust_to_speed(double thrust) const {
    if (thrust > 0.0) {
      return static_cast<int8_t>(std::clamp(thrust * 20.0, -100.0, 100.0));
    }
    return static_cast<int8_t>(std::clamp(thrust * 25.0, -100.0, 100.0));
  }

  void finish_mission(const std::string & reason) {
    mission_active_ = false;
    publish_zero_thrust();
    publish_status("Mission complete: " + reason);
    RCLCPP_INFO(get_logger(), "Mission complete: %s", reason.c_str());
  }

  void publish_status(const std::string & message) {
    auto msg = std_msgs::msg::String();
    msg.data = message;
    status_publisher_->publish(msg);
  }

 public:
  void stop_all_motion_on_shutdown() {
    mission_active_ = false;
    publish_zero_thrust();
  }

 private:
  rclcpp::Publisher<snappy_interfaces::msg::ThrusterCommand>::SharedPtr motor_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<snappy_interfaces::msg::Pose>::SharedPtr state_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<MissionStep> mission_steps_;
  std::size_t mission_step_index_ = 0;
  bool step_started_ = false;
  bool mission_active_ = true;
  double default_speed_ = 35.0;
  double command_rate_hz_ = 20.0;
  std::string state_topic_;

  rclcpp::Time step_started_at_;
  rclcpp::Time depth_hold_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  std::chrono::steady_clock::time_point kill_deadline_;
  bool kill_latched_ = false;
  double current_depth_ = 0.0;
  double current_yaw_ = 0.0;
  double depth_hold_elapsed_ = 0.0;
  snappy_interfaces::msg::Pose current_position_;

  ThrusterAllocator thruster_allocator_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeBasedController>();
  rclcpp::spin(node);
  node->stop_all_motion_on_shutdown();
  rclcpp::shutdown();
  return 0;
}
