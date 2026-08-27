#include <functional>
#include <iostream>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <opencv2/opencv.hpp>

class CameraCalibration : public rclcpp::Node {
    public: 
        CameraCalibration() : Node("camera_calibration") {
            image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&CameraCalibration::image_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Camera Calibration started");
            
            last_capture_time_ = this->now();

            for(int y = 0; y < board_height_; y++) {
                for(int x = 0; x < board_width_; x++) {
                    object_template_.emplace_back(x * square_size_, y * square_size_, 0.0f);
                }
            }
        }
    
    private:
        const int board_width_ = 7;
        const int board_height_ = 7;
        const float square_size_ = 0.025f;
        const size_t required_images_ = 25;

        cv::Mat camera_matrix_;
        cv::Mat distortion_coefficients_;
        cv::Size board_size_{
            board_width_,
            board_height_
        };
        
        bool calibrated_ = false;
        bool saved_test_image_ = false;
        rclcpp::Time last_capture_time_;

        std::vector<cv::Point3f> object_template_;
        std::vector<std::vector<cv::Point3f>> object_points_;
        std::vector<std::vector<cv::Point2f>> image_points_;

        cv::Size image_size_;

        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            if(calibrated_) {
                if(!saved_test_image_) {
                    cv_bridge::CvImagePtr cv_ptr;

                    try {
                        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                    }
                    catch (const cv_bridge::Exception &e){
                        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
                        return;
                    }

                    cv::Mat frame = cv_ptr->image;
                    cv::Mat undistorted_frame;

                    cv::undistort(
                        frame,
                        undistorted_frame,
                        camera_matrix_,
                        distortion_coefficients_
                    );

                    cv::imwrite("/tmp/raw_frame.jpg", frame);
                    cv::imwrite("/tmp/undistorted_frame.jpg", undistorted_frame);
                    RCLCPP_INFO(this->get_logger(), "Saved test images");
                    saved_test_image_ = true;                    
                } 
                return;
            }

            cv_bridge::CvImagePtr cv_ptr;
            
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            }
            catch (const cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
                return;
            }
            
            cv::Mat frame = cv_ptr->image;
            cv::Mat gray;

            cv::cvtColor(
                frame,
                gray,
                cv::COLOR_BGR2GRAY
            );

            image_size_ = gray.size();
            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(gray, board_size_, corners);

            if(!found) {
                RCLCPP_INFO(this->get_logger(), "Chessboard not detected");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Chessboard detected: %zu corners", corners.size());

            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), 
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));

            double elapsed = (this->now() - last_capture_time_).seconds();
            if(elapsed < 2.0) {
                return;
            }

            image_points_.push_back(corners);
            object_points_.push_back(object_template_);
            last_capture_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Calibration sample accepted: %zu / %zu", image_points_.size(), required_images_);

            if(!calibrated_ && image_points_.size() >= required_images_) {
                calibrate();
            }
        }

        void calibrate() {
            std::vector<cv::Mat> rotation_vectors;
            std::vector<cv::Mat> translation_vectors;

            double rms = cv::calibrateCamera(object_points_, image_points_, image_size_, camera_matrix_, distortion_coefficients_, rotation_vectors, translation_vectors);
        
            RCLCPP_INFO(this->get_logger(), "Calibration complete");
            RCLCPP_INFO(this->get_logger(), "RMS reprojection error: %.6f", rms);

            std::cout << "Camera matrix:\n" << camera_matrix_ << std::endl;
            std::cout << "Distortion coefficients:\n" << distortion_coefficients_ << std::endl;
            calibrated_ = true;

            // THESE CALIBRATION SETTINGS ARE NOT SAVED
            // NEED TO BE SAVED IN YAML FILE
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CameraCalibration>());

    rclcpp::shutdown();

    return 0;
}