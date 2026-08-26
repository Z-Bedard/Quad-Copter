#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <libcamera/libcamera.h>

class CameraNode : public rclcpp::Node {
    public: 
        CameraNode() : Node("camera_node") {
            RCLCPP_INFO (this->get_logger(), "Starting camera node");

            camera_manager_ = std::make_unique<libcamera::CameraManager>();
            int ret = camera_manager_->start();

            if(ret) {
                RCLCPP_ERROR(this->get_logger(), "Failed to start libcamera CameraManager");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "libcamera CameraMaanger Started");

            const auto &cameras = camera_manager_->cameras();

            if(cameras.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No cameras detected");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Detected %zu camera(s)", cameras.size());

            for(const auto &camera : cameras) {
                RCLCPP_INFO(this->get_logger(), "Camera: %s", camera->id().c_str());
            }

            camera_ = cameras[0];
            RCLCPP_INFO(this->get_logger(), "Selected camera: %s", camera_->id().c_str());
        }

        ~CameraNode() {
            if(camera_manager_) {
                camera_manager_->stop();
            }
        }
    
    private:
        std::unique_ptr<libcamera::CameraManager>camera_manager_;
        std::shared_ptr<libcamera::Camera>camera_;    
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CameraNode>());

    rclcpp::shutdown();

    return 0;
}