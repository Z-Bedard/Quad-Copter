#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class BridgeSim : public rclcpp::Node{
    public:
        BridgeSim() : Node("bridge_sim") {
            command_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>("/drone/cmd_attitude", 10, std::bind(&BridgeSim::command_callback, this, std::placeholders::_1));

            attitude_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/drone/attitude", 10);

            battery_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/drone/battery_voltage", 10);

            timer_ = this->create_wall_timer(100ms, std::bind(&BridgeSim::publish_telemetry, this));
        }
        

    private:
        double target_roll_ = 0.0;
        double target_pitch_ = 0.0;
        double target_yaw_ = 0.0;

        double current_roll_ = 0.0;
        double current_pitch_ = 0.0;
        double current_yaw_ = 0.0;

        double response_gain = 0.1;

        void command_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
            target_roll_ = msg->x;
            target_pitch_ = msg->y;
            target_yaw_ = msg->z;

            RCLCPP_INFO(this->get_logger(), "New Command In: \n Roll: %.2f\n Pitch: %.2f\n Yaw: %.2f", msg->x, msg->y, msg->z);
        }

        void publish_telemetry() {
            geometry_msgs::msg::Vector3 attitude_msg;

            current_roll_ += response_gain*(target_roll_ - current_roll_);
            current_pitch_ += response_gain*(target_pitch_ - current_pitch_);
            current_yaw_ += response_gain*(target_yaw_ - current_yaw_);

            attitude_msg.x = current_roll_;
            attitude_msg.y = current_pitch_;
            attitude_msg.z = current_yaw_;

            attitude_publisher_->publish(attitude_msg);

            std_msgs::msg::Float32 battery_msg;
            battery_msg.data = 11.7;

            battery_publisher_->publish(battery_msg);

            RCLCPP_INFO(this->get_logger(), "Current Attitude -> Roll: %.2f Pitch: %.2f Yaw: %.2f", current_roll_, current_pitch_, current_yaw_);
        }

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr command_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<BridgeSim>());

    rclcpp::shutdown();

    return 0;
}