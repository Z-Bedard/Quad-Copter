#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;

class GuidanceNode : public rclcpp::Node {
    public: 
        GuidanceNode() : Node("guidance_node") {
            command_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/drone/cmd_attitude", 10);

            timer_ = this->create_wall_timer(500ms, std::bind(&GuidanceNode::publish_command, this));
        }
    
        private:
            void publish_command() {
                geometry_msgs::msg::Vector3 msg;

                msg.x = 5.0;
                msg.y = -3.0;
                msg.z = 0.0;

                command_publisher_ -> publish(msg);

                RCLCPP_INFO(this->get_logger(), "Command Sent -> \n Roll: %.2f\n Pitch: %.2f\n Yaw: %.2f", msg.x, msg.y, msg.z);
            }

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<GuidanceNode>());

    rclcpp::shutdown();

    return 0;
}