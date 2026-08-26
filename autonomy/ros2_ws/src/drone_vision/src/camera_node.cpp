#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>

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

            RCLCPP_INFO(this->get_logger(), "libcamera CameraManager Started");

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
        
            ret = camera_->acquire();

            if(ret) {
                RCLCPP_ERROR(this->get_logger(), "Failed to acquire camera");
                return;
            }
            camera_acquired = true;

            RCLCPP_INFO(this->get_logger(), "Camera acquired");
        
            config_ = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
            if(!config_) {
                RCLCPP_ERROR(this->get_logger(), "Failed to generate camera configuration");
                return;
            }

            libcamera::StreamConfiguration & stream_config = config_->at(0);

            stream_config.size.width = 640;
            stream_config.size.height = 480;

            auto status = config_->validate();
            if(status == libcamera::CameraConfiguration::Invalid) {
                RCLCPP_ERROR(this->get_logger(), "Camera config is invalid");
                return;
            }

            if(status == libcamera::CameraConfiguration::Adjusted) {
                RCLCPP_WARN(this->get_logger(), "Camera config was adjusted by libcamera");
            }

            RCLCPP_INFO(this->get_logger(), "Configured stream: %ux%u", stream_config.size.width, stream_config.size.height);

            ret = camera_->configure(config_.get());
            if(ret) {
                RCLCPP_ERROR(this->get_logger(), "Failed to configure camera");
                return;
            }

            stream_ = stream_config.stream();

            allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
            ret = allocator_->allocate(stream_);

            if(ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to allocate frame buffers");
                return;
            }

            const auto & buffers = allocator_->buffers(stream_);

            for(const auto & buffer : buffers) {
                std::unique_ptr<libcamera::Request> request = camera_->createRequest();
                if(!request) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create request");
                    return;
                }

                ret = request->addBuffer(stream_, buffer.get());
                if(ret < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to add buffer to request");
                    return;
                }

                requests_.push_back(std::move(request));
            }

            camera_->requestCompleted.connect(this, &CameraNode::request_complete);

            ret = camera_->start();

            if(ret) {
                RCLCPP_ERROR(this->get_logger(), "Failed to start camera");
                return;
            }
            camera_started = true;

            for(auto & request : requests_) {
                ret = camera_->queueRequest(request.get());
                if(ret < 0){
                    RCLCPP_ERROR(this->get_logger(), "Failed to queue request");
                    return;
                }
            }

            RCLCPP_INFO(this->get_logger(), "Camera capture started");
        }

        ~CameraNode() {
            if(camera_){
                camera_->stop();
                camera_->release();
            }
            if(camera_manager_) {
                camera_manager_->stop();
            }
        }
    
    private:
        std::unique_ptr<libcamera::CameraManager>camera_manager_;
        std::shared_ptr<libcamera::Camera>camera_;    

        std::unique_ptr<libcamera::CameraConfiguration> config_;
        std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;

        libcamera::Stream *stream_ = nullptr;

        std::vector<std::unique_ptr<libcamera::Request>> requests_;

        bool camera_acquired = false;
        bool camera_started = true;

        void request_complete(libcamera::Request * request) {
            if(request->status() == libcamera::Request::RequestCancelled){
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Frame Received");

            request->reuse(libcamera::Request::ReuseBuffers);
            int ret = camera_->queueRequest(request);

            if(ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to requeue request");
            }
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CameraNode>());

    rclcpp::shutdown();

    return 0;
}