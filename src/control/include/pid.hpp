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

    public:
        PID(float Kp, float Ki, float Kd); // Constructor
        void set_target(float target);
        float update(float current); // Return the magnitude of movement
};

#endif
