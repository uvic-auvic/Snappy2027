#include "pid.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

PID::PID(float Kp, float Ki, float Kd) {
    if (!std::isfinite(Kp) || !std::isfinite(Ki) || !std::isfinite(Kd))
        throw std::invalid_argument("PID gains must be finite");
    this->Kp_ = Kp;
    this->Ki_ = Ki;
    this->Kd_ = Kd;
    this->target_ = 0.0f;
    this->integral_ = 0.0f;
    this->prev_err_ = 0.0f;
    this->prev_time_ = std::chrono::steady_clock::now();
    this->MIN = -100.0f;
    this->MAX = 100.0f;
}

void PID::set_target(float target) {
    if (!std::isfinite(target)) throw std::invalid_argument("PID target must be finite");
    if (target == target_) return;
    target_ = target;
    reset_derivative();
}

void PID::reset_derivative() {
    derivative_ready_ = false;
}

float PID::update(float current) {
    // Get time
    auto cur_time = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(cur_time - prev_time_).count();
    if (dt < 0.001f) { // Protect against division by 0 on first call
        dt = 0.001f;
    }
    prev_time_ = cur_time;

    return update(current, dt);
}

float PID::update(float current, float dt) {
    if (!std::isfinite(current)) throw std::invalid_argument("PID input must be finite");
    if (!std::isfinite(dt) || dt <= 0.0f) {
        throw std::invalid_argument("PID dt must be finite and positive");
    }
    const float err = target_ - current;

    // Proportional term
    float p_term = Kp_ * err;

    // Integral term
    integral_before_update_ = integral_;
    integral_ += err * dt;
    if (integral_ < MIN) {
        integral_ = MIN;
    } else if (integral_ > MAX) {
        integral_ = MAX;
    }
    float i_term = Ki_ * integral_;
    pending_integral_change_ = Ki_ * (integral_ - integral_before_update_);

    // Derivative term (on-error)
    float d_term = derivative_ready_ ? Kd_ * ((err - prev_err_) / dt) : 0.0f;
    derivative_ready_ = true;
    prev_err_ = err;

    // Clamp output
    float output = p_term + i_term + d_term;
    if (!std::isfinite(output)) throw std::runtime_error("PID arithmetic overflow");
    requested_output_ = output;
    if (output < MIN) {
        output = MIN;
    } else if (output > MAX) {
        output = MAX;
    }

    // Reject integration that pushes farther into this PID's own output clamp.
    // Allocator feedback can subsequently reject integration into motor limits.
    const float pending = pending_integral_change_;
    if ((pending > 0.0f && requested_output_ > output) ||
        (pending < 0.0f && requested_output_ < output)) {
        integral_ = integral_before_update_;
        pending_integral_change_ = 0.0f;
    }
    return output;
}

void PID::apply_output_feedback(float achieved_output) {
    if (!std::isfinite(achieved_output)) throw std::invalid_argument("PID feedback must be finite");
    const float residual = requested_output_ - achieved_output;
    // Allow integration that unwinds saturation. Ignore floating-point noise.
    if ((pending_integral_change_ > 0.0f && residual > 1e-5f) ||
        (pending_integral_change_ < 0.0f && residual < -1e-5f)) {
        integral_ = integral_before_update_;
    }
    pending_integral_change_ = 0.0f;
}
