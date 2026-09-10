// Depth sensor node. Reads newline-delimited "D <metres>" lines from an Arduino
// (Bar02 pressure sensor) over a USB serial port and republishes the latest
// reading as a Float32 on depth_data, which the controller and planner consume.
// Canonical, nonblocking reads return complete lines when available. Each timer
// callback drains a bounded amount of buffered input and uses the newest sample.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <vector>
#include <cmath>
#include <limits>

class DepthSensorNode : public rclcpp::Node
{
public:
    // Open the configured port at 115200 8N1 and start the 10 Hz read timer.
    DepthSensorNode() : Node("depth_sensor_node"), serial_fd_(-1)
    {
        const std::string serial_port = declare_parameter("serial_port", std::string("/dev/ttyUSB0"));
        // Publishes a plain float — your state estimator reads this directly
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("depth_data", 10);

        // Avoid blocking during open on modem control lines. Canonical mode
        // buffers incomplete lines; EAGAIN simply means no full line is ready.
        serial_fd_ = open(serial_port.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0) {
            throw std::runtime_error("Failed to open depth port " + serial_port + ": " + strerror(errno));
        }

        // FIX (bug 3): Zero-initialise tty and check tcgetattr return value so we
        // never pass a garbage struct to tcsetattr if the fd is not a real tty.
        struct termios tty{};
        if (tcgetattr(serial_fd_, &tty) < 0) {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            throw std::runtime_error("Depth serial configuration failed");
        }

        // Arduino sketch uses 115200 baud — must match
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        // this enables the receiver and ignores the control lines
        tty.c_cflag |= (CLOCAL | CREAD);
        // this ensures no parity bit
        tty.c_cflag &= ~PARENB;
        // this ensures 1 stop bit
        tty.c_cflag &= ~CSTOPB;

        // this ensures 8 data bits
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;

        // Canonical mode: read() returns one complete line at a time (\n terminated)
        // This prevents partial reads and is exactly how the Python node works —
        // kernel buffers bytes until a newline is received.
        tty.c_lflag |= ICANON;
        // FIX (bug 6): Clear all echo flags (ECHOK and ECHONL were previously left
        // set alongside ECHO and ECHOE, violating raw-mode convention).
        tty.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL | ISIG);

        // disable the software flow control (XON/XOFF) and other special characters
        // IGNCR strips Arduino CRLF's carriage return; LF terminates the line.
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
        tty.c_iflag |= IGNCR;

        // output processing
        // raw output — no \n → \r\n translation on send
        tty.c_oflag &= ~OPOST;

        // FIX (bug 3): Check tcsetattr return value.
        if (tcsetattr(serial_fd_, TCSANOW, &tty) < 0) {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            throw std::runtime_error("Depth serial configuration failed");
        }

        // Verify nonblocking reads remain enabled after terminal configuration.
        int flags = fcntl(serial_fd_, F_GETFL, 0);
        if (flags < 0 || fcntl(serial_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
            RCLCPP_ERROR(this->get_logger(), "fcntl F_SETFL failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            throw std::runtime_error("Depth serial configuration failed");
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DepthSensorNode::read_and_publish, this));

        RCLCPP_INFO(this->get_logger(), "Depth sensor on %s at 115200 baud", serial_port.c_str());
    }

    // Close the serial port if it was opened.
    ~DepthSensorNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    // Parses "D 1.23" (with optional trailing \r/\n/space) → 1.23.
    // Returns false if the format doesn't match.
    bool parse_depth_line(const std::string & line, double & depth_out)
    {
        // Expected format from Arduino: "D <value>"
        const std::string prefix = "D ";

        if (line.rfind(prefix, 0) != 0) {
            return false;  // Line doesn't start with "D "
        }

        std::string trimmed = line;
        // Strip trailing whitespace / \r / \n
        while (!trimmed.empty() && (trimmed.back() == '\r' || trimmed.back() == '\n' || trimmed.back() == ' ')) {
            trimmed.pop_back();
        }

        if (trimmed.size() <= prefix.size()) {
            return false;
        }

        // Extract the numeric part
        std::string num_str = trimmed.substr(prefix.size());

        try {
            size_t consumed = 0;
            depth_out = std::stod(num_str, &consumed);
            return consumed == num_str.size() && std::isfinite(depth_out) &&
                std::abs(depth_out) <= std::numeric_limits<float>::max();
        } catch (...) {
            return false;
        }
    }

    // Timer callback: drain the serial buffer, parse the newest complete line,
    // and publish it on depth_data. Drops stale lines so we never lag behind.
    void read_and_publish()
    {
        if (serial_fd_ < 0) return;

        // FIX (bugs 2 & 5): Accumulate all complete lines from this drain pass,
        // then walk backwards to find and publish only the freshest valid one.
        // Previously, latest_line was overwritten with the raw buffer on each
        // read(), which (a) picked the first "D " match in a multi-line buffer
        // (oldest, not newest) and (b) discarded all complete lines if the final
        // read() returned a partial line with no parseable prefix.
        std::vector<std::string> lines;
        char buffer[256];

        // Bound work per callback even when a device continuously streams data.
        for (int reads = 0; reads < 64; ++reads) {
            int bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0';
                // In canonical mode each read() delivers at most one complete line,
                // but split on \n defensively in case of any buffering edge cases.
                std::string chunk(buffer, bytes_read);
                std::string::size_type start = 0;
                std::string::size_type pos;
                while ((pos = chunk.find('\n', start)) != std::string::npos) {
                    lines.push_back(chunk.substr(start, pos - start + 1));
                    start = pos + 1;
                }
                // Any remainder after the last \n is a partial line — discard it;
                // canonical mode will deliver the rest on the next read().
                continue;
            }
            if (bytes_read < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;  // buffer empty
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Serial read failed: %s", strerror(errno));
                return;
            }
            break;  // bytes_read == 0, EOF
        }

        if (lines.empty()) return;

        // Walk lines newest-first; publish the first one that parses successfully.
        for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
            double depth = 0.0;
            if (parse_depth_line(*it, depth)) {
                auto message = std_msgs::msg::Float32();
                message.data = depth;
                publisher_->publish(message);
                //RCLCPP_INFO(this->get_logger(), "Depth: %.4f m", depth);
                return;
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_fd_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<DepthSensorNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("depth_sensor_node"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
