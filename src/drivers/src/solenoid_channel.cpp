// solenoid_node.cpp
//
// Controls two solenoids (left / right torpedo release). Each has its own
// trigger topic. On a rising edge (0 -> 1) the node opens that solenoid and
// starts a one-shot timer to close it again after OPEN_DURATION.
//
// GPIO pin info is NOT hardcoded. It's exposed as ROS2 parameters, so
// whoever runs this on the actual Jetson can supply the correct gpiochip
// path and line offset (found via `gpioinfo` on their machine) without
// touching or recompiling this file. Defaults below are a best guess from
// the J12 header pinout table (subject to change -- see param descriptions).
//
// Requires libgpiod (C++ bindings):
//   sudo apt install libgpiod-dev
// CMakeLists.txt needs:
//   target_link_libraries(solenoid_node gpiodcxx)

#include <std_msgs/msg/bool.hpp>
#include <gpiod.hpp>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// Holds the state for one solenoid channel (left or right). Bundling this
// together means the open/close logic below is written once and reused for
// both solenoids instead of being duplicated.
struct SolenoidChannel
{
  std::string name;             // "left" or "right", used for logging
  gpiod::line line;             // the actual GPIO line handle, requested at startup
  bool is_open = false;
  rclcpp::TimerBase::SharedPtr close_timer;
};

class SolenoidNode : public rclcpp::Node
{
public:
  SolenoidNode() : Node("solenoid_node")
  {
    left_.name = "left";
    right_.name = "right";

    // --- Declare parameters so pin info can be supplied at launch time ---
    // Defaults are a starting guess (see pinout table); whoever runs this
    // should confirm with `gpioinfo` on the actual Jetson and override via
    // a launch file / YAML / command line if these are wrong.
    this->declare_parameter<std::string>("left.gpio_chip", "/dev/gpiochip0");
    this->declare_parameter<int>("left.line_offset", 144);   // header pin 7 (GPIO09), best guess
    this->declare_parameter<std::string>("right.gpio_chip", "/dev/gpiochip0");
    this->declare_parameter<int>("right.line_offset", 85);   // header pin 15 (GPIO12), best guess

    setup_gpio_line(left_, "left.gpio_chip", "left.line_offset");
    setup_gpio_line(right_, "right.gpio_chip", "right.line_offset");

    left_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "solenoid_left_trigger",
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { trigger_callback(msg, left_); }
    );

    right_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "solenoid_right_trigger",
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { trigger_callback(msg, right_); }
    );

    RCLCPP_INFO(this->get_logger(), "Solenoid node started, waiting for triggers.");
  }

private:
  // Opens the configured gpiochip, grabs the configured line, and requests
  // it as an output (initially low / closed). Runs once at startup per
  // channel. If the chip/line params are wrong, this will throw and the
  // node will fail to start with an error naming the bad chip/offset --
  // that's intentional, so a bad pin config is loud and obvious.
  void setup_gpio_line(SolenoidChannel & ch, const std::string & chip_param, const std::string & offset_param)
  {
    std::string chip_path = this->get_parameter(chip_param).as_string();
    int offset = this->get_parameter(offset_param).as_int();

    gpiod::chip chip(chip_path);
    ch.line = chip.get_line(offset);
    ch.line.request({
      "solenoid_node",                         // consumer name, shows up in gpioinfo
      gpiod::line_request::DIRECTION_OUTPUT,
      0
    }, 0);  // initial value: 0 (closed)

    RCLCPP_INFO(this->get_logger(), "%s solenoid using %s line %d",
      ch.name.c_str(), chip_path.c_str(), offset);
  }

  void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg, SolenoidChannel & ch)
  {
    // Only react on a rising edge. Ignore repeats, and ignore a 1 that
    // arrives while this channel's solenoid is already open.
    if (msg->data && !ch.is_open)
    {
      open_channel(ch);
    }
  }

  void open_channel(SolenoidChannel & ch)
  {
    ch.is_open = true;
    ch.line.set_value(1);
    RCLCPP_INFO(this->get_logger(), "%s solenoid opened.", ch.name.c_str());

    // One-shot timer: fires once after OPEN_DURATION, then cancels itself.
    // Captures a reference to this channel so left/right close independently.
    ch.close_timer = this->create_wall_timer(
      OPEN_DURATION,
      [this, &ch]() {
        close_channel(ch);
        ch.close_timer->cancel();
      }
    );
  }

  void close_channel(SolenoidChannel & ch)
  {
    ch.line.set_value(0);
    ch.is_open = false;
    RCLCPP_INFO(this->get_logger(), "%s solenoid closed.", ch.name.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_sub_;
  SolenoidChannel left_;
  SolenoidChannel right_;

  static constexpr auto OPEN_DURATION = 500ms;  // TODO: set actual open time
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SolenoidNode>());
  rclcpp::shutdown();
  return 0;
}
