#ifndef SNAPPY_ROV_INPUT_HPP
#define SNAPPY_ROV_INPUT_HPP

#include <cstdint>
#include <vector>

// Pure input/state logic. Time arguments use steady-clock seconds, not ROS time.
struct RovConfig {
    int forward_axis = 5;
    int reverse_axis = 2;
    int strafe_axis = 0;
    int yaw_axis = 3;
    int dpad_axis = 7;  // Set to -1 to use D-pad buttons instead.
    int dpad_up_button = -1;
    int dpad_down_button = -1;
    int deadman_button = 4;
    int stop_button = 1;
    double trigger_released = 1.0;
    double trigger_pressed = -1.0;
    bool invert_strafe = true;  // Body +y is RIGHT; Linux stick +x is LEFT.
    bool invert_yaw = true;     // Body +yaw is RIGHT in the z-down frame.
    bool invert_dpad = false;
    double deadzone = 0.1;
    double max_horizontal = 0.3;
    double max_yaw = 0.3;
    double depth_increment = 0.25;
    double min_depth = 0.0;
    double max_depth = -1.0;  // Unconfigured: no depth control or motor output.
    double surface_tolerance = 0.05;
    double joy_timeout = 0.5;
    double depth_timeout = 0.5;
    double command_timeout = 0.5;

    void validate() const;
    bool depth_limits_configured() const;
};

struct RovRequest {
    double surge = 0.0;
    double sway = 0.0;
    double yaw = 0.0;
    double depth_target = 0.0;
    bool depth_enabled = false;
    bool manual_enabled = false;
    bool deadman_pressed = false;
    bool emergency_stop = false;  // Raw Circle state; controller owns the latch.
};

class RovInput {
public:
    explicit RovInput(const RovConfig & config);
    bool receive(const std::vector<float> & axes, const std::vector<int32_t> & buttons,
                 double time);
    void update_depth(double measured_depth, bool healthy);
    void set_stop_latched(bool latched);
    RovRequest request(double time);
    void invalidate_joy();

private:
    RovConfig config_;
    RovRequest request_;
    bool joy_valid_ = false;
    bool released_ = false;
    bool up_ = true;
    bool down_ = true;
    bool depth_healthy_ = false;
    bool stop_latched_ = false;
    bool local_stop_ = false;
    double measured_depth_ = 0.0;
    double last_joy_ = 0.0;
};

#endif
