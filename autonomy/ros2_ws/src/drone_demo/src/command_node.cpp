#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class CommandNode : public rclcpp::Node {

    public:
    CommandNode() : Node("command_node") {
        subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>("/drone/cmd_attitude", 10, std::bind(&CommandNode::command_callback, this, std::placeholders::_1));
    }

    private:
    void command_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Command received -> \n Roll: %.2f\n Pitch: %.2f\n Yaw: %.2f", msg->x, msg->y, msg->z);
    }

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CommandNode>());

    rclcpp::shutdown();

    return 0;
}