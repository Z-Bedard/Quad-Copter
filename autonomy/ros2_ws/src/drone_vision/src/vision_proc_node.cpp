#include <algorithm>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

class VisionProc : public rclcpp::Node {
    public:
        VisionProc() : Node("vision_proc_node") {
            image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&VisionProc::image_callback, this, std::placeholders::_1));
            
            matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);

            orb_ = cv::ORB::create(1000);
        }

    private:
        // Temp Hardcoded calibration values before YAML file with actual calibration values
        cv::Mat camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            336.0698913378847, 0.0, 329.6543631430312,
            0.0, 336.4258361270623, 206.5706502606883,
            0.0, 0.0, 1.0);

        cv::Mat distortion_coefficients_ = (cv::Mat_<double>(1, 5) <<
            -0.4641815694275584,
            0.4817709594610811,
            0.01016659010261606,
            0.004722940452727565,
            -0.4214890718676393);

        cv::Ptr<cv::ORB> orb_;   
        cv::Ptr<cv::BFMatcher> matcher_;

        std::vector<cv::KeyPoint> prev_keypoints_;
        cv::Mat prev_descriptors_;

        bool have_prev_frame_ = false;

        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            cv_bridge::CvImagePtr cv_ptr;

            try {
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            }
            catch (const cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
                return;
            }

            cv::Mat frame = cv_ptr->image;

            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;

            orb_->detectAndCompute(frame, cv::noArray(), keypoints, descriptors);
            if(have_prev_frame_ && !prev_descriptors_.empty() && !descriptors.empty()) {
                std::vector<cv::DMatch> matches;
                matcher_->match(prev_descriptors_, descriptors, matches);

                if(matches.size() < 10) {
                    RCLCPP_WARN(this->get_logger(), "Not enough matches: %zu", matches.size());
                } else {
                    std::sort(matches.begin(), matches.end(),
                        [](const cv::DMatch &a, const cv::DMatch &b) {
                            return a.distance < b.distance;
                        });
                    
                    size_t keep_count = matches.size() / 2;
                    std::vector<cv::DMatch> good_matches(matches.begin(), matches.begin() + keep_count);
                    
                    std::vector<cv::Point2f> prev_points;
                    std::vector<cv::Point2f> curr_points;

                    for(const auto &match : good_matches) {
                        prev_points.push_back(prev_keypoints_[match.queryIdx].pt);
                        curr_points.push_back(keypoints[match.trainIdx].pt);
                    }
                    
                    std::vector<cv::Point2f> prev_points_undistorted;
                    std::vector<cv::Point2f> curr_points_undistorted;
                    
                    cv::undistortPoints(prev_points, prev_points_undistorted, camera_matrix_, distortion_coefficients_);
                    cv::undistortPoints(curr_points, curr_points_undistorted, camera_matrix_, distortion_coefficients_);    

                    if(prev_points_undistorted.size() >= 8){
                        cv::Mat inlier_mask;

                        cv::Mat essential_matrix = cv::findEssentialMat(prev_points_undistorted, curr_points_undistorted, 1.0, cv::Point2d(0.0, 0.0), cv::RANSAC, 0.999, 0.003, inlier_mask);
                        if(!essential_matrix.empty()) {
                            int inlier_count = cv::countNonZero(inlier_mask);
                            RCLCPP_INFO(this->get_logger(), "RANSAC inliers: %d / %zu", inlier_count, prev_points_undistorted.size());
                        
                            cv::Mat rotation;
                            cv::Mat translation;

                            int pose_inliers = cv::recoverPose(essential_matrix, prev_points_undistorted, curr_points_undistorted, rotation, translation, 1.0, cv::Point2d(0.0, 0.0), inlier_mask);
                            RCLCPP_INFO(this->get_logger(), "Pose inliers: %d", pose_inliers);
                            std::cout<< "Rotation:\n" << rotation << std::endl;
                            std::cout<< "Translation direction:\n" << translation << std::endl;
                        }
                    }

                    RCLCPP_INFO(this->get_logger(), "Features: %zu | Matches: %zu | Good Matches: %zu | Raw pairs: %zu | Undistorted pairs: %zu", keypoints.size(), matches.size(), good_matches.size(), prev_points.size(), prev_points_undistorted.size());
                    if(!prev_points_undistorted.empty()) {
                        RCLCPP_INFO(this->get_logger(), "Raw prev: (%.2f, %.2f) | Undistorted: (%.4f, %.4f)", prev_points[0].x, prev_points[0].y, prev_points_undistorted[0].x, prev_points_undistorted[0].y);
                    }


                }
            }
            prev_keypoints_ = keypoints;
            prev_descriptors_ = descriptors.clone();
            have_prev_frame_ = true;
        }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};

int main (int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<VisionProc>());

    rclcpp::shutdown();

    return 0;
}