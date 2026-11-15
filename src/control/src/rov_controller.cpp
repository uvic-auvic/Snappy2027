#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "snappy_interfaces/msg/rov_command.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rov_parameters.hpp"
#include <limits>

using namespace std::chrono_literals;

class RovController : public rclcpp::Node {
public:
    RovController() : Node("rov_controller"), config_(load_rov_config(*this)),
        input_(config_), depth_(config_) {
        publisher_ = create_publisher<snappy_interfaces::msg::RovCommand>("/rov/command", 1);
        joy_ = create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Joy & msg) {
                sync_depth();
                const double age = (now() - rclcpp::Time(msg.header.stamp)).seconds();
                const bool fresh = std::isfinite(age) && age >= -0.1 && age < config_.joy_timeout;
                const bool valid = input_.receive(msg.axes, msg.buttons,
                    fresh ? rov_steady_seconds() : std::numeric_limits<double>::quiet_NaN());
                if (!valid) {
                    input_.invalidate_joy();
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Invalid or stale Joy: manual control requires a fresh L1 release and press");
                }
                publish();  // Circle and L1 release do not wait for the heartbeat.
            });
        pressure_ = create_subscription<std_msgs::msg::Float32>("/depth_data", 1,
            [this](const std_msgs::msg::Float32 & msg) {
                depth_.pressure(msg.data, rov_steady_seconds());
                sync_depth();
            });
        state_ = create_subscription<snappy_interfaces::msg::Pose>("/state_estimator/state", 1,
            [this](const snappy_interfaces::msg::Pose & msg) {
                depth_.state(msg, rov_steady_seconds());
                sync_depth();
            });
        stop_ = create_subscription<std_msgs::msg::Bool>("/rov/emergency_stop",
            rclcpp::QoS(1).transient_local(), [this](const std_msgs::msg::Bool & msg) {
                input_.set_stop_latched(msg.data);
            });
        timer_ = create_wall_timer(20ms, [this]() { sync_depth(); publish(); });
        if (!config_.depth_limits_configured())
            RCLCPP_WARN(get_logger(), "max_depth_m is unset; depth control and all thrust are disabled");
    }

private:
    void sync_depth() { input_.update_depth(depth_.depth(), depth_.healthy(rov_steady_seconds())); }
    void publish() {
        const auto request = input_.request(rov_steady_seconds());
        snappy_interfaces::msg::RovCommand msg;
        msg.header.stamp = now();
        msg.header.frame_id = "base_link";
        msg.surge = request.surge;
        msg.sway = request.sway;
        msg.yaw = request.yaw;
        msg.depth_target = request.depth_target;
        msg.depth_enabled = request.depth_enabled;
        msg.manual_enabled = request.manual_enabled;
        msg.deadman_pressed = request.deadman_pressed;
        msg.emergency_stop = request.emergency_stop;
        publisher_->publish(msg);
    }
    const RovConfig config_;
    RovInput input_;
    RovDepthHealth depth_;
    rclcpp::Publisher<snappy_interfaces::msg::RovCommand>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pressure_;
    rclcpp::Subscription<snappy_interfaces::msg::Pose>::SharedPtr state_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RovController>());
    rclcpp::shutdown();
    return 0;
}
