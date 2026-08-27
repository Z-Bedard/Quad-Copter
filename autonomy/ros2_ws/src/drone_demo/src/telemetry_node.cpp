#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class TelemetryNode : public rclcpp::Node
{
public:
    TelemetryNode() : Node("telemetry_node") {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/drone/battery_voltage", 10);

        timer_ = this->create_wall_timer(500ms, std::bind(&TelemetryNode::publish_voltage, this));
    }

private:
    void publish_voltage() {
        std_msgs::msg::Float32 message;

        // Fake battery voltage for now.
        message.data = 11.7;

        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Battery voltage: %.2f V", message.data);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<TelemetryNode>());

    rclcpp::shutdown();

    return 0;
}