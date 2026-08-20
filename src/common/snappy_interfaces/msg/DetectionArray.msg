# Array of object detections from a single frame
std_msgs/Header header
uint32 inference_time_ms   # Inference time in milliseconds
ObjectDetection[] detections
