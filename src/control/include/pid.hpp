#ifndef PID_H
#define PID_H

#include <chrono>

class PID {
    private:
        float Kp_;
        float Ki_;
        float Kd_;
        float target_;
        float integral_;
        float prev_err_;
        std::chrono::steady_clock::time_point prev_time_;
        float MIN;
        float MAX;
        bool derivative_ready_ = false;
        float integral_before_update_ = 0.0f;
        float pending_integral_change_ = 0.0f;
        float requested_output_ = 0.0f;

    public:
        PID(float Kp, float Ki, float Kd); // Constructor
        void set_target(float target);
        float update(float current); // Return the magnitude of movement
        float update(float current, float dt); // Explicit step for deterministic tests
        // Keep learned steady-state compensation; suppress D for the next sample.
        void reset_derivative();
        // Call once after allocation, in PID output units (exclude feedforward).
        void apply_output_feedback(float achieved_output);
};

#endif
