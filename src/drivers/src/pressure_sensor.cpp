// Depth sensor node. Reads newline-delimited "D <metres>" lines from an Arduino
// (Bar02 pressure sensor) over a USB serial port and republishes the latest
// reading as a Float32 on depth_data, which the controller and planner consume.
// The serial port is opened in canonical, blocking mode so each read() returns
// one complete line; the timer drains the buffer and keeps only the freshest
// sample.
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

class DepthSensorNode : public rclcpp::Node
{
public:
    // Open and configure /dev/ttyUSB0 (115200 8N1, canonical) and start the
    // 10 Hz read timer. If the port can't be opened the node stays up but idle.
    DepthSensorNode() : Node("depth_sensor_node"), serial_fd_(-1)
    {
        // Publishes a plain float — your state estimator reads this directly
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("depth_data", 10);

        // FIX (bugs 1 & 4): Open WITHOUT O_NONBLOCK. Canonical mode needs blocking
        // reads so that read() waits for a full \n-terminated line rather than
        // returning EAGAIN immediately on a partially-arrived line.
        serial_fd_ = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY);
        if (serial_fd_ < 0) {
            //RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/ttyUSB0");
            return;
        }

        // FIX (bug 3): Zero-initialise tty and check tcgetattr return value so we
        // never pass a garbage struct to tcsetattr if the fd is not a real tty.
        struct termios tty{};
        if (tcgetattr(serial_fd_, &tty) < 0) {
            //RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return;
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

        // Canonical mode: read() returns one complete line at a time (\n terminated)
        // This prevents partial reads and is exactly how the Python node works —
        // kernel buffers bytes until a newline is received.
        tty.c_lflag |= ICANON;
        // FIX (bug 6): Clear all echo flags (ECHOK and ECHONL were previously left
        // set alongside ECHO and ECHOE, violating raw-mode convention).
        tty.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL | ISIG);

        // disable the software flow control (XON/XOFF) and other special characters
        // FIX (bug 7): Also clear ICRNL so that \r from Arduino Serial.println()
        // \r\n endings is stripped by the kernel before the line is delivered,
        // preventing stray \r bytes from confusing prefix matching and parse logic.
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);

        // output processing
        // raw output — no \n → \r\n translation on send
        tty.c_oflag &= ~OPOST;

        // FIX (bug 3): Check tcsetattr return value.
        if (tcsetattr(serial_fd_, TCSANOW, &tty) < 0) {
            //RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return;
        }

        // FIX (bugs 1 & 4): O_NONBLOCK is a file-descriptor flag — tcsetattr cannot
        // clear it. We need it set during open() on some BSDs for non-blocking
        // modem-control negotiation, but for the drain loop we want non-blocking
        // reads (to detect an empty kernel buffer). So we re-enable O_NONBLOCK via
        // fcntl now that the port is fully configured and canonical mode is active.
        // This is distinct from the original bug: canonical mode + O_NONBLOCK is
        // only safe once the port is configured; the original code opened with
        // O_NONBLOCK and never verified termios was applied correctly first.
        int flags = fcntl(serial_fd_, F_GETFL, 0);
        if (flags < 0 || fcntl(serial_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
            //RCLCPP_ERROR(this->get_logger(), "fcntl F_SETFL failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DepthSensorNode::read_and_publish, this));

        //RCLCPP_INFO(this->get_logger(), "Depth sensor node started on /dev/ttyUSB0 at 115200 baud");
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
            depth_out = std::stod(num_str);
            return true;
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

        while (true) {
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
                //RCLCPP_ERROR(this->get_logger(), "Error reading from serial: %s", strerror(errno));
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
    auto node = std::make_shared<DepthSensorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
