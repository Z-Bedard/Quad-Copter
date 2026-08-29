#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

class SerialBridge : public rclcpp::Node {
    public:
        SerialBridge() : Node("serial_bridge_node") {
            attitude_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/drone/fc/attitude_deg", 10);
            gyro_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/drone/fc/gyro_dps", 10);
            armed_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/drone/fc/armed", 10);
            failsafe_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/drone/fc/failsafe", 10);
            throttle_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/drone/fc/throttle_us", 10);

            open_serial_port();
            serial_timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&SerialBridge::read_serial, this));
        }

        ~SerialBridge() {
            if(serial_fd_ >= 0) {
                close(serial_fd_);
            }
        }

        private:
            void open_serial_port() {
                serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
                if(serial_fd_ < 0) {
                    RCLCPP_FATAL(this->get_logger(), "Failed to open /dev/ttyUSB0: %s", std::strerror(errno));
                    throw std::runtime_error("Could not open serial port");
                }

                termios tty{};
                if(tcgetattr(serial_fd_, &tty) != 0) {
                    close(serial_fd_);
                    serial_fd_ = -1;
                    throw std::runtime_error("Failed to read serial configuration");
                }

                cfsetispeed(&tty, B115200);
                cfsetospeed(&tty, B115200);

                tty.c_cflag &= ~PARENB;
                tty.c_cflag &= ~CSTOPB;
                tty.c_cflag &= ~CSIZE;
                tty.c_cflag |= CS8;

                tty.c_cflag &= ~CRTSCTS;
                tty.c_cflag |= CREAD | CLOCAL;

                tty.c_lflag &= ~ICANON;
                tty.c_lflag &= ~ECHO;
                tty.c_lflag &= ~ECHOE;
                tty.c_lflag &= ~ISIG;

                tty.c_iflag &= ~(IXON | IXOFF | IXANY);

                tty.c_oflag &= ~OPOST;

                tty.c_cc[VMIN] = 0;
                tty.c_cc[VTIME] = 0;

                if(tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
                    close(serial_fd_);
                    serial_fd_ = -1;

                    throw std::runtime_error("Failed to configure serial port");
                }

                tcflush(serial_fd_, TCIFLUSH);
                RCLCPP_INFO(this->get_logger(), "Connected to ESP32 on /dev/ttyUSB0 at 115200 baud");
            }

            void read_serial(){
                char buffer[256];
                ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer));
                if(bytes_read <= 0) {
                    return;
                }

                receive_buffer.append(buffer, static_cast<size_t>(bytes_read));
                size_t newline_position;

                while((newline_position = receive_buffer_.find('\n')) != std::string::npos) {
                    std::string line = receive_buffeR_.substr(0, newline_position);
                    receive_buffer_.erase(0, newline_position + 1);
                    if(!line.empty() && line.back() == '\r') {
                        line.pop_back();
                    }
                    parse_telemetry(line);
                }
            }

            void parse_telemetry(const std::string& line) {
                if(line.rfind("TEL,", 0) != 0) {
                    return;
                }

                std::stringstream stream(line);
                std::string token;
                std::vector<std::string> fields;

                while(std::getline(stream, token, ',')) {
                    fields.push_back(token);
                }

                if(fields.size() != 12) {
                    RCLCPP_WARN(this->get_logger(), "Invalid telemetry packet: %s", line.c_str());
                    return;
                }

                try {
                    const double roll_deg = std::stod(fields[1]);
                    const double pitch_deg = std::stod(fields[2]);

                    const double gyro_x_dps = std::stod(fields[3]);
                    const double gyro_y_dps = std::stod(fields[4]);
                    const double gyro_z_dps = std::stod(fields[5]);

                    const bool armed = std::stoi(fields[6]) != 0;
                    const bool failsafe = std::stoi(fields[7]) != 0;

                    const int throttle_us = std::stoi(fields[8]);

                    geometry_msgs::msg::Vector3 attitude_msg;
                    attitude_msg.x = roll_deg;
                    attitude_msg.y = pitch_deg;
                    attitude_msg.z = 0.0;

                    geometry_msgs::msg::Vector3 gyro_msg;
                    gyro_msg.x = gyro_x_dps;
                    gyro_msg.y = gyro_y_dps;
                    gyro_msg.z = gyro_z_dps;

                    std_msgs::msg::Bool armed_msg;
                    armed_msg.data = armed;

                    std_msgs::msg::Bool failsafe_msg;
                    failsafe_msg.data = failsafe;

                    std_msgs::msg::Int32 throttle_msg;
                    throttle_msg.data = throttle_us;

                    attitude_publisher_->publish(attitude_msg);
                    gyro_publisher_->publish(gyro_msg);
                    armed_publisher_->publish(armed_msg);
                    failsafe_publisher_->publish(failsafe_msg);
                    throttle_publisher_->publish(throttle_msg);
                }
                catch (const std::exception& e) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Failed to parse telemetry: %s",
                        line.c_str());
                }                
            }

        int serial_fd_ = -1;
        std::string receive_buffer_;

        rclcpp::TimerBase::SharedPtr serial_timer_;

        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gyro_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr failsafe_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr throttle_publisher_;
}

int main(int argc, char * argv[] {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SerialBridge>());

    rclcpp::shutdown();

    return 0;
})