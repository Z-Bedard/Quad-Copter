#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class CameraSim : public rclcpp::Node {
    public:
        CameraSim() : Node("camera_sim_node") {
            image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);

            camera_.open(0);
            if(!camera_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Couldn't open cam");
            }

            timer_ = this->create_wall_timer(33ms, std::bind(&CameraSim::publish_frame, this));
        }

    private:
        void publish_frame() {
            cv::Mat frame;

            camera_ >> frame;

            if(frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Empty frame received");
                return;
            }

            auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        
            image_publisher_->publish(*message);
        }

    cv::VideoCapture camera_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CameraSim>());

    rclcpp::shutdown();

    return 0;
}