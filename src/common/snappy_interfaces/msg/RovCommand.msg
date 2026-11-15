# Atomic request: the existing Task is an autonomous pose goal and cannot express
# manual effort together with a depth setpoint and the input safety state.
std_msgs/Header header
# Normalized body effort: +x forward, +y right, +yaw right (z down).
float64 surge
float64 sway
float64 yaw
# Positive metres down in the estimator's world frame.
float64 depth_target
bool depth_enabled
bool manual_enabled
# Raw button states allow the controller to reject reset while either is held.
bool deadman_pressed
bool emergency_stop
