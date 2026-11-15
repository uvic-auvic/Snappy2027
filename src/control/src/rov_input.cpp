#include "rov_input.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
void require(bool valid, const char * message) {
    if (!valid) throw std::invalid_argument(message);
}
bool positive(double value) { return std::isfinite(value) && value > 0.0; }
bool fraction(double value) { return positive(value) && value <= 1.0; }
double stick(double value, double deadzone, bool inverted) {
    if (std::abs(value) <= deadzone) return 0.0;
    const double scaled = std::copysign((std::abs(value) - deadzone) / (1.0 - deadzone), value);
    return inverted ? -scaled : scaled;
}
}

bool RovConfig::depth_limits_configured() const {
    return std::isfinite(max_depth) && max_depth > min_depth;
}

void RovConfig::validate() const {
    require(forward_axis >= 0 && reverse_axis >= 0 && strafe_axis >= 0 && yaw_axis >= 0,
            "Movement axis indices must be nonnegative");
    require(deadman_button >= 0 && stop_button >= 0 && deadman_button != stop_button,
            "Deadman and emergency-stop buttons must be distinct nonnegative indices");
    require(dpad_axis >= -1 && (dpad_axis >= 0 ||
            (dpad_up_button >= 0 && dpad_down_button >= 0 && dpad_up_button != dpad_down_button)),
            "Configure a D-pad axis or two distinct D-pad buttons");
    require(std::isfinite(trigger_released) && std::isfinite(trigger_pressed) &&
            std::abs(trigger_pressed - trigger_released) > 1e-6 &&
            std::abs(trigger_released) <= 1.0 && std::abs(trigger_pressed) <= 1.0,
            "Trigger endpoints must be distinct finite values in [-1, 1]");
    require(std::isfinite(deadzone) && deadzone >= 0.0 && deadzone < 1.0,
            "deadzone must be in [0, 1)");
    require(fraction(max_horizontal) && fraction(max_yaw), "Thrust limits must be in (0, 1]");
    require(positive(depth_increment), "depth_increment_m must be positive and finite");
    require(std::isfinite(min_depth) && min_depth >= 0.0 && std::isfinite(max_depth) &&
            (max_depth == -1.0 || max_depth > min_depth),
            "Depth limits must satisfy 0 <= minimum < maximum, or maximum=-1 (disabled)");
    require(std::isfinite(surface_tolerance) && surface_tolerance >= 0.0,
            "surface_tolerance_m must be finite and nonnegative");
    require(positive(joy_timeout) && positive(depth_timeout) && positive(command_timeout),
            "All timeouts must be positive and finite");
}

RovInput::RovInput(const RovConfig & config) : config_(config) { config_.validate(); }

void RovInput::invalidate_joy() {
    joy_valid_ = false;
    released_ = false;
    up_ = down_ = true;  // Require D-pad neutral before accepting another press.
    request_.surge = request_.sway = request_.yaw = 0.0;
    request_.manual_enabled = false;
}

void RovInput::update_depth(double measured_depth, bool healthy) {
    healthy = healthy && config_.depth_limits_configured() && std::isfinite(measured_depth) &&
        measured_depth >= config_.min_depth && measured_depth <= config_.max_depth;
    if (healthy && !depth_healthy_) request_.depth_target = measured_depth;
    if (!healthy) invalidate_joy();
    measured_depth_ = measured_depth;
    depth_healthy_ = healthy;
}

void RovInput::set_stop_latched(bool latched) {
    // Only a true -> false transition acknowledges the controller's explicit reset.
    if (stop_latched_ && !latched) {
        local_stop_ = false;
        depth_healthy_ = false;  // Recapture measured depth on the next healthy sample.
        invalidate_joy();
    }
    stop_latched_ = latched;
    if (latched) invalidate_joy();
}

bool RovInput::receive(const std::vector<float> & axes,
                       const std::vector<int32_t> & buttons, double time) {
    // Honor Circle even if the rest of the packet is malformed.
    if (static_cast<size_t>(config_.stop_button) < buttons.size() &&
        buttons[config_.stop_button] != 0) {
        request_.emergency_stop = true;
        local_stop_ = true;
    }
    const int max_axis = std::max({config_.forward_axis, config_.reverse_axis,
        config_.strafe_axis, config_.yaw_axis, config_.dpad_axis});
    const int max_button = std::max({config_.deadman_button, config_.stop_button,
        config_.dpad_axis < 0 ? config_.dpad_up_button : -1,
        config_.dpad_axis < 0 ? config_.dpad_down_button : -1});
    const bool valid = std::isfinite(time) && axes.size() > static_cast<size_t>(max_axis) &&
        buttons.size() > static_cast<size_t>(max_button) &&
        std::all_of(axes.begin(), axes.end(), [](float v) { return std::isfinite(v) && std::abs(v) <= 1.0f; }) &&
        std::all_of(buttons.begin(), buttons.end(), [](int32_t v) { return v == 0 || v == 1; });
    if (!valid) { invalidate_joy(); return false; }
    if (joy_valid_ && time - last_joy_ >= config_.joy_timeout) invalidate_joy();
    joy_valid_ = true;
    last_joy_ = time;
    request_.emergency_stop = buttons[config_.stop_button] != 0;
    request_.deadman_pressed = buttons[config_.deadman_button] != 0;
    if (!request_.deadman_pressed) released_ = true;

    const bool enabled = released_ && request_.deadman_pressed && depth_healthy_ &&
        !local_stop_ && !stop_latched_;
    request_.manual_enabled = enabled;
    auto trigger = [&](int index) {
        return std::clamp((axes[index] - config_.trigger_released) /
            (config_.trigger_pressed - config_.trigger_released), 0.0, 1.0);
    };
    request_.surge = enabled ? config_.max_horizontal *
        (trigger(config_.forward_axis) - trigger(config_.reverse_axis)) : 0.0;
    request_.sway = enabled ? config_.max_horizontal *
        stick(axes[config_.strafe_axis], config_.deadzone, config_.invert_strafe) : 0.0;
    request_.yaw = enabled ? config_.max_yaw *
        stick(axes[config_.yaw_axis], config_.deadzone, config_.invert_yaw) : 0.0;

    const double dpad = config_.dpad_axis >= 0 ? axes[config_.dpad_axis] *
        (config_.invert_dpad ? -1.0 : 1.0) : 0.0;
    const bool up = config_.dpad_axis >= 0 ? dpad > 0.5 : buttons[config_.dpad_up_button] != 0;
    const bool down = config_.dpad_axis >= 0 ? dpad < -0.5 : buttons[config_.dpad_down_button] != 0;
    if (enabled && up != down) {
        if (up && !up_ && measured_depth_ > config_.min_depth + config_.surface_tolerance)
            request_.depth_target = std::max(config_.min_depth, request_.depth_target - config_.depth_increment);
        if (down && !down_)
            request_.depth_target = std::min(config_.max_depth, request_.depth_target + config_.depth_increment);
    }
    up_ = up;
    down_ = down;
    return true;
}

RovRequest RovInput::request(double time) {
    if (!joy_valid_ || time - last_joy_ >= config_.joy_timeout) invalidate_joy();
    auto result = request_;
    result.depth_enabled = depth_healthy_ && !local_stop_ && !stop_latched_;
    return result;
}
