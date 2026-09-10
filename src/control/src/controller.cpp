// Onboard pose controller. Every motor command passes through the kill latch.
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include "snappy_interfaces/msg/pose.hpp"
#include "snappy_interfaces/msg/task.hpp"
#include "snappy_interfaces/msg/thruster_command.hpp"
#include "control_math.hpp"
#include "pid.hpp"
#include "thruster_allocator.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

class Controller : public rclcpp::Node {
public:
    Controller() : Node("controller") {
        enable_xy_ = parameter("enable_xy_control", false);
        wait_for_task_ = parameter("wait_for_task", false);
        kill_timeout_ = positive_parameter("kill_timeout_s", 30.0);
        state_timeout_ = positive_parameter("state_timeout_s", 0.5);
        task_timeout_ = positive_parameter("task_timeout_s", 60.0);
        position_tolerance_ = positive_parameter("position_tolerance_m", 0.1);
        orientation_tolerance_ = positive_parameter("orientation_tolerance_rad", 0.0872664626);
        settle_time_ = positive_parameter("settle_time_s", 0.5);
        max_settle_speed_ = positive_parameter("settle_speed_m_s", 0.1);
        max_settle_rate_ = positive_parameter("settle_rate_rad_s", 0.1745329252);
        heave_feedforward_ = parameter("heave_feedforward", 5.0);
        if (!std::isfinite(heave_feedforward_)) throw std::invalid_argument("heave_feedforward must be finite");
        const std::array<std::string, 6> names = {"pid_x", "pid_y", "pid_z", "pid_roll", "pid_pitch", "pid_yaw"};
        for (size_t i = 0; i < names.size(); ++i) {
            const auto gains = vector_parameter(names[i], {0.0, 0.0, 0.0});
            pids_[i] = PID(gains[0], gains[1], gains[2]);
            enabled_gains_[i] = gains[0] != 0 || gains[1] != 0 || gains[2] != 0;
        }
        const auto position = vector_parameter("target_position", {0.0, 0.0, 1.0});
        target_position_ = Eigen::Vector3d(position[0], position[1], position[2]);
        const double roll = parameter("target_roll", 0.0);
        const double pitch = parameter("target_pitch", 0.0);
        const double yaw = parameter("target_yaw", 0.0);
        target_orientation_ = snappy_control::orientation_from_rpy(roll, pitch, yaw);
        if (!enable_xy_ && target_position_.head<2>().norm() > 1e-9)
            throw std::invalid_argument("Nonzero X/Y target requires enable_xy_control and valid position feedback");
        check_attitude_gains(roll, pitch, yaw);

        configuration_.resize(6, 8);
        configuration_ <<
            0, 0, 1, 0, 0, 0, 1, 0,
            -1, 0, 0, 0, 1, 0, 0, 0,
            0, 1, 0, 1, 0, 1, 0, 1,
            .1301, .1653, 0, .1653, -.1301, -.1648, 0, -.1648,
            0, .3024, -.0158, -.2977, 0, -.2977, -.0159, .3024,
            -.3041, 0, -.2739, 0, -.3121, 0, .2734, 0;
        allocator_ = ThrusterAllocator(configuration_, -4.0f, 5.0f);
        motor_pub_ = create_publisher<snappy_interfaces::msg::ThrusterCommand>("/motor_cmd", rclcpp::QoS(1).best_effort());
        status_pub_ = create_publisher<std_msgs::msg::String>("/controller/status", rclcpp::QoS(1).transient_local());
        done_pub_ = create_publisher<std_msgs::msg::Int32>("/controller/task_done", 10);
        target_pub_ = create_publisher<snappy_interfaces::msg::Pose>("/controller/target", rclcpp::QoS(1).transient_local());
        requested_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("/controller/wrench_requested", 10);
        allocated_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("/controller/wrench_allocated", 10);
        task_sub_ = create_subscription<snappy_interfaces::msg::Task>("/planner/task", rclcpp::QoS(1).transient_local(),
            [this](const snappy_interfaces::msg::Task& task) { accept_task(task); });
        abort_sub_ = create_subscription<std_msgs::msg::String>("/planner/abort", rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::String& reason) { latch_stop("planner abort: " + reason.data); });
        state_sub_ = create_subscription<snappy_interfaces::msg::Pose>("/state_estimator/state", rclcpp::QoS(1),
            [this](const snappy_interfaces::msg::Pose& state) { accept_state(state); });
        started_ = previous_tick_ = Clock::now();
        publish_target();
        set_status(wait_for_task_ ? "waiting_for_task" : "waiting_for_state");
        RCLCPP_WARN(get_logger(), "Kill deadline: %.1f seconds from controller startup", kill_timeout_);
        if (!enabled_gains_[3] || !enabled_gains_[4])
            RCLCPP_WARN(get_logger(), "Zero roll/pitch gains: nonzero targets on those disabled axes will be rejected");
        timer_ = create_wall_timer(20ms, [this]() {
            try { tick(); }
            catch (const std::exception& error) { latch_stop(error.what()); }
        });
    }

private:
    // Startup-only settings must not report successful runtime changes that are ignored.
    template<class T> T parameter(const std::string& name, const T& fallback) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        return declare_parameter<T>(name, fallback, descriptor);
    }
    double positive_parameter(const std::string& name, double fallback) {
        const double value = parameter(name, fallback);
        if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(name + " must be finite and positive");
        return value;
    }
    std::vector<double> vector_parameter(const std::string& name, const std::vector<double>& fallback) {
        const auto values = parameter(name, fallback);
        if (values.size() != 3) throw std::invalid_argument(name + " requires exactly three numbers");
        for (double value : values)
            if (!std::isfinite(value)) throw std::invalid_argument(name + " must contain finite numbers");
        return values;
    }
    static double seconds(Clock::time_point end, Clock::time_point start) {
        return std::chrono::duration<double>(end - start).count();
    }
    void check_attitude_gains(double roll, double pitch, double yaw) const {
        const std::array<double, 3> angles = {roll, pitch, yaw};
        const std::array<std::string, 3> names = {"roll", "pitch", "yaw"};
        for (size_t i = 0; i < angles.size(); ++i)
            if (std::abs(angles[i]) > 1e-9 && !enabled_gains_[i + 3])
                throw std::invalid_argument("Nonzero " + names[i] + " target has zero PID gains");
    }
    void set_status(const std::string& status) {
        if (status == status_) return;
        status_ = status;
        std_msgs::msg::String message;
        message.data = status;
        status_pub_->publish(message);
        RCLCPP_INFO(get_logger(), "%s", status.c_str());
    }
    void publish_zero() {
        snappy_interfaces::msg::ThrusterCommand message{};
        message.header.stamp = now();
        message.header.frame_id = "base_link";
        message.thruster_mask = 255;
        message.thrust_pct.fill(0);
        motor_pub_->publish(message);
    }
    void latch_stop(const std::string& reason) {
        if (!killed_) {
            killed_ = true;
            set_status("failed:" + std::to_string(task_ ? task_->seq : -1) + ":" + reason);
            RCLCPP_ERROR(get_logger(), "All thrusters latched at zero: %s", reason.c_str());
        }
        publish_zero();
    }
    void publish_target() {
        snappy_interfaces::msg::Pose message;
        message.position.x = target_position_.x(); message.position.y = target_position_.y(); message.position.z = target_position_.z();
        message.orientation.x = target_orientation_.x(); message.orientation.y = target_orientation_.y();
        message.orientation.z = target_orientation_.z(); message.orientation.w = target_orientation_.w();
        target_pub_->publish(message);
    }
    void acknowledge() {
        std_msgs::msg::Int32 message;
        message.data = task_->seq;
        done_pub_->publish(message);
    }
    void accept_task(const snappy_interfaces::msg::Task& task) {
        if (killed_) return;
        try {
            const Eigen::Vector3d position(task.x, task.y, task.z);
            if (!position.allFinite() || task.seq < 0) throw std::invalid_argument("Invalid task position or sequence");
            const auto orientation = snappy_control::orientation_from_rpy(task.roll, task.pitch, task.yaw);
            if (task_ && task.seq < task_->seq) return;
            if (task_ && task.seq == task_->seq) {
                if ((position - target_position_).norm() > 1e-9 || orientation.angularDistance(target_orientation_) > 1e-9)
                    throw std::invalid_argument("Task sequence reused with different targets; restart mission/controller together");
                if (task_completed_) acknowledge();
                return; // Retransmission must not restart settling, PID, or deadlines.
            }
            if (!enable_xy_ && position.head<2>().norm() > 1e-9)
                throw std::invalid_argument("X/Y task cannot run while enable_xy_control is false");
            check_attitude_gains(task.roll, task.pitch, task.yaw);
            for (int i = 0; i < 3; ++i)
                if (std::abs(position[i] - target_position_[i]) > 1e-9) pids_[i].reset_derivative();
            if (orientation.angularDistance(target_orientation_) > 1e-9)
                for (int i = 3; i < 6; ++i) pids_[i].reset_derivative();
            target_position_ = position; // Includes planner depth, in metres.
            target_orientation_ = orientation;
            task_ = task;
            task_completed_ = false;
            task_started_ = Clock::now();
            settled_since_.reset();
            publish_target();
            set_status("accepted:" + std::to_string(task.seq));
            RCLCPP_INFO(get_logger(), "Task %d: position [%.3f, %.3f, %.3f], RPY [%.3f, %.3f, %.3f] rad",
                        task.seq, task.x, task.y, task.z, task.roll, task.pitch, task.yaw);
        } catch (const std::exception& error) {
            latch_stop("task " + std::to_string(task.seq) + ": " + error.what());
        }
    }
    void accept_state(const snappy_interfaces::msg::Pose& state) {
        const Eigen::Vector3d position(state.position.x, state.position.y, state.position.z);
        const Eigen::Quaterniond quaternion(state.orientation.w, state.orientation.x, state.orientation.y, state.orientation.z);
        if (!position.allFinite() || !quaternion.coeffs().allFinite() ||
            !std::isfinite(quaternion.norm()) || quaternion.norm() < 1e-6) {
            if (active_) latch_stop("invalid estimator state");
            return;
        }
        const auto time = Clock::now();
        const auto orientation = quaternion.normalized();
        if (last_state_) {
            const double dt = seconds(time, *last_state_);
            if (dt > 1e-6) {
                Eigen::Vector3d delta = position - current_position_;
                if (!enable_xy_) delta.head<2>().setZero();
                speed_ = delta.norm() / dt;
                angular_rate_ = orientation.angularDistance(current_orientation_) / dt;
            }
        }
        current_position_ = position;
        current_orientation_ = orientation;
        last_state_ = time;
        // Settling requires fresh measurements, not repeated timer ticks on a frozen pose.
        if (task_ && !task_completed_) {
            Eigen::Vector3d error = target_position_ - current_position_;
            if (!enable_xy_) error.head<2>().setZero();
            const bool within = error.norm() < position_tolerance_ &&
                current_orientation_.angularDistance(target_orientation_) < orientation_tolerance_ &&
                speed_ < max_settle_speed_ && angular_rate_ < max_settle_rate_;
            if (!within) settled_since_.reset();
            else if (!settled_since_) settled_since_ = time;
        }
    }
    void publish_wrench(const Eigen::VectorXd& wrench,
        const rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr& publisher) {
        geometry_msgs::msg::WrenchStamped message;
        message.header.stamp = now(); message.header.frame_id = "base_link";
        message.wrench.force.x = wrench[0]; message.wrench.force.y = wrench[1]; message.wrench.force.z = wrench[2];
        message.wrench.torque.x = wrench[3]; message.wrench.torque.y = wrench[4]; message.wrench.torque.z = wrench[5];
        publisher->publish(message);
    }
    void tick() {
        const auto time = Clock::now();
        const double dt = std::max(1e-3, seconds(time, previous_tick_));
        previous_tick_ = time;
        if (killed_) { publish_zero(); return; }
        if (seconds(time, started_) >= kill_timeout_) { latch_stop("kill timer expired"); return; }
        if (task_ && !task_completed_ && seconds(time, task_started_) >= task_timeout_) {
            latch_stop("task timeout"); return;
        }
        if (!last_state_ || seconds(time, *last_state_) > state_timeout_) {
            settled_since_.reset();
            if (active_) latch_stop("estimator state lost or stale");
            else { set_status("waiting_for_state"); publish_zero(); }
            return;
        }
        if (wait_for_task_ && !task_) { set_status("waiting_for_task"); publish_zero(); return; }
        active_ = true;
        if (!task_completed_) set_status(task_ ? "tracking:" + std::to_string(task_->seq) : "holding_fixed_target");

        Eigen::Vector3d position_error = target_position_ - current_position_;
        if (!enable_xy_) position_error.head<2>().setZero();
        const auto angle_error = snappy_control::body_rotation_error(current_orientation_, target_orientation_);
        Eigen::Vector3d world_force;
        for (int i = 0; i < 3; ++i) world_force[i] = pids_[i].update(-position_error[i], dt);
        if (!enable_xy_) world_force.head<2>().setZero();
        world_force.z() += heave_feedforward_;
        // Depth and buoyancy are world-vertical, even while the sub is tilted.
        Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
        wrench.head<3>() = current_orientation_.conjugate() * world_force;
        for (int i = 0; i < 3; ++i) wrench[i + 3] = pids_[i + 3].update(-angle_error[i], dt);
        Eigen::VectorXd priority = Eigen::VectorXd::Zero(6);
        priority.head<3>() = current_orientation_.conjugate() * Eigen::Vector3d(0, 0, world_force.z());
        const auto forces = allocator_.allocate_with_priority(wrench, priority);
        const Eigen::VectorXd achieved = configuration_ * forces;
        const Eigen::Vector3d achieved_world_force = current_orientation_ * Eigen::Vector3d(achieved.head<3>());
        pids_[0].apply_output_feedback(achieved_world_force.x());
        pids_[1].apply_output_feedback(achieved_world_force.y());
        pids_[2].apply_output_feedback(achieved_world_force.z() - heave_feedforward_);
        for (int i = 3; i < 6; ++i) pids_[i].apply_output_feedback(achieved[i]);
        snappy_interfaces::msg::ThrusterCommand message{};
        message.header.stamp = now(); message.header.frame_id = "base_link"; message.thruster_mask = 255;
        for (size_t i = 0; i < message.thrust_pct.size(); ++i)
            message.thrust_pct[i] = snappy_control::thrust_to_command(forces[i]);
        motor_pub_->publish(message);
        publish_wrench(wrench, requested_pub_);
        publish_wrench(achieved, allocated_pub_);
        // A failed control calculation must never acknowledge a successful task.
        if (task_ && !task_completed_ && settled_since_ && seconds(*last_state_, *settled_since_) >= settle_time_) {
            task_completed_ = true;
            acknowledge();
            set_status("completed:" + std::to_string(task_->seq));
        }
    }

    std::array<PID, 6> pids_ = {PID(0,0,0), PID(0,0,0), PID(0,0,0), PID(0,0,0), PID(0,0,0), PID(0,0,0)};
    std::array<bool, 6> enabled_gains_{};
    Eigen::MatrixXd configuration_;
    ThrusterAllocator allocator_;
    Eigen::Vector3d current_position_ = Eigen::Vector3d::Zero(), target_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond current_orientation_ = Eigen::Quaterniond::Identity(), target_orientation_ = Eigen::Quaterniond::Identity();
    Clock::time_point started_, previous_tick_, task_started_;
    std::optional<Clock::time_point> last_state_, settled_since_;
    std::optional<snappy_interfaces::msg::Task> task_;
    double kill_timeout_, state_timeout_, task_timeout_, position_tolerance_, orientation_tolerance_, settle_time_;
    double max_settle_speed_, max_settle_rate_, heave_feedforward_;
    double speed_ = std::numeric_limits<double>::infinity(), angular_rate_ = std::numeric_limits<double>::infinity();
    bool enable_xy_, wait_for_task_, active_ = false, killed_ = false, task_completed_ = false;
    std::string status_;
    rclcpp::Publisher<snappy_interfaces::msg::ThrusterCommand>::SharedPtr motor_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr done_pub_;
    rclcpp::Publisher<snappy_interfaces::msg::Pose>::SharedPtr target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr requested_pub_, allocated_pub_;
    rclcpp::Subscription<snappy_interfaces::msg::Task>::SharedPtr task_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr abort_sub_;
    rclcpp::Subscription<snappy_interfaces::msg::Pose>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<Controller>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("controller"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
