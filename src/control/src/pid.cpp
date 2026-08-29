#include "pid.hpp"

PID::PID(float Kp, float Ki, float Kd) {
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
    target_ = target;
    integral_ = 0; // Reset integral
}

float PID::update(float current) {
    // Get error
    float err = target_ - current;

    // Get time
    auto cur_time = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(cur_time - prev_time_).count();
    if (dt < 0.001f) { // Protect against division by 0 on first call
        dt = 0.001f;
    }
    prev_time_ = cur_time;

    // Proportional term
    float p_term = Kp_ * err;

    // Integral term
    integral_ += err * dt;
    if (integral_ < MIN) {
        integral_ = MIN;
    } else if (integral_ > MAX) {
        integral_ = MAX;
    }
    float i_term = Ki_ * integral_;

    // Derivative term (on-error)
    float d_term = Kd_ * ((err - prev_err_) / dt);
    prev_err_ = err;

    // Clamp output
    float output = p_term + i_term + d_term;
    if (output < MIN) {
        output = MIN;
    } else if (output > MAX) {
        output = MAX;
    }

    return output;
}
