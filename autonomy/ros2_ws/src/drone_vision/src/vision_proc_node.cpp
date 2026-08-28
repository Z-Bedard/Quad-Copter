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
            
            matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false);

            orb_ = cv::ORB::create(1000);
        }

    private:
        const int MIN_POSE_INLIERS = 15;
        const double MIN_POSE_INLIER_RATIO = 0.5;
        const double MIN_KEYFRAME_DISPLACEMENT_PX = 8.0;
        const double MAX_KEYFRAME_DISPLACEMENT_PX = 80.0;

        const int MAX_TRACKING_FAILURES = 3;
        const size_t MIN_GOOD_MATCHES_FOR_TRACKING = 20;
        const size_t MIN_FEATURES_FOR_KEYFRAME = 100;
        const float MATCH_RATIO_THRESHOLD = 0.75f;

        int tracking_failure_count_ = 0;

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

        std::vector<cv::KeyPoint> keyframe_keypoints_;
        cv::Mat keyframe_descriptors_;

        bool have_keyframe_ = false;

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
            if(!have_keyframe_) {
                if(descriptors.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Cannot create initial keyframe: no descriptors");
                    return;
                }
                keyframe_keypoints_ = keypoints;
                keyframe_descriptors_ = descriptors.clone();
                have_keyframe_ = true;

                RCLCPP_INFO(this->get_logger(), "Initial keyframe created");
                return;
            }
            
            if(have_keyframe_ && !keyframe_descriptors_.empty() && !descriptors.empty()) {
                std::vector<std::vector<cv::DMatch>> knn_matches;
                matcher_->knnMatch(keyframe_descriptors_, descriptors, knn_matches, 2);
                
                std::vector<cv::DMatch> good_matches;
                for (const auto &match_pair : knn_matches) {
                    if(match_pair.size() < 2) {
                        continue;
                    }

                    const cv::DMatch &best_match = match_pair[0];
                    const cv::DMatch &second_best_match = match_pair[1];
                    if(best_match.distance < MATCH_RATIO_THRESHOLD * second_best_match.distance) {
                        good_matches.push_back(best_match);
                    }
                }

                
                if(good_matches.size() < MIN_GOOD_MATCHES_FOR_TRACKING) {
                    if(tracking_failure_count_ < MAX_TRACKING_FAILURES){
                        tracking_failure_count_++;
                    }
                    RCLCPP_WARN(this->get_logger(), "Tracking weak: %zu good matches | Failure %d / %d", good_matches.size(), tracking_failure_count_, MAX_TRACKING_FAILURES);

                    if(tracking_failure_count_ >= MAX_TRACKING_FAILURES){
                        if(keypoints.size() >= MIN_FEATURES_FOR_KEYFRAME){
                            tracking_failure_count_ = 0;
                            keyframe_keypoints_ = keypoints;
                            keyframe_descriptors_ = descriptors.clone();
                            RCLCPP_WARN(this->get_logger(), "Tracking lost - resetting");
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Tracking lost, Current frame has too few features: %zu", keypoints.size());
                        }
                    }
                    return;
                }
                tracking_failure_count_ = 0;

                std::vector<cv::Point2f> keyframe_points;
                std::vector<cv::Point2f> curr_points;

                for(const auto &match : good_matches) {
                    keyframe_points.push_back(keyframe_keypoints_[match.queryIdx].pt);
                    curr_points.push_back(keypoints[match.trainIdx].pt);
                }

                std::vector<double> displacements;
                for(size_t i = 0; i <keyframe_points.size(); ++i) {
                    double dx = curr_points[i].x - keyframe_points[i].x;
                    double dy = curr_points[i].y - keyframe_points[i].y;
                    double displacement = std::sqrt(dx * dx + dy * dy);
                    displacements.push_back(displacement);
                }
                
                std::sort(displacements.begin(), displacements.end());
                double median_displacement = displacements[displacements.size() / 2];
                RCLCPP_INFO(this->get_logger(), "Median displacement from keyframe: %.2f px", median_displacement);
                if(median_displacement < MIN_KEYFRAME_DISPLACEMENT_PX) {
                    RCLCPP_INFO(this->get_logger(), "Keeping current keyframe");
                    return;
                }
                if(median_displacement > MAX_KEYFRAME_DISPLACEMENT_PX) {
                    RCLCPP_WARN(this->get_logger(), "Keyframe is stale (%.2f px) - resetting", median_displacement);
                    if(keypoints.size() >= MIN_FEATURES_FOR_KEYFRAME) {
                        keyframe_keypoints_ = keypoints;
                        keyframe_descriptors_ = descriptors.clone();
                        tracking_failure_count_ = 0;
                        RCLCPP_WARN(this->get_logger(), "Reference keyframe reset");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Current frame has too few features for reset: %zu", keypoints.size());
                    }
                    return;
                }

                std::vector<cv::Point2f> keyframe_points_undistorted;
                std::vector<cv::Point2f> curr_points_undistorted;
                
                cv::undistortPoints(keyframe_points, keyframe_points_undistorted, camera_matrix_, distortion_coefficients_);
                cv::undistortPoints(curr_points, curr_points_undistorted, camera_matrix_, distortion_coefficients_);    

                if(keyframe_points_undistorted.size() >= 8){
                    cv::Mat inlier_mask;

                    cv::Mat essential_matrix = cv::findEssentialMat(keyframe_points_undistorted, curr_points_undistorted, 1.0, cv::Point2d(0.0, 0.0), cv::RANSAC, 0.999, 0.003, inlier_mask);
                    if(!essential_matrix.empty()) {
                        int inlier_count = cv::countNonZero(inlier_mask);
                        RCLCPP_INFO(this->get_logger(), "RANSAC inliers: %d / %zu", inlier_count, keyframe_points_undistorted.size());
                        if(inlier_count == 0) {
                            RCLCPP_WARN(this->get_logger(), "RANSAC returned zero inliers");
                            return;
                        }

                        cv::Mat rotation;
                        cv::Mat translation;

                        int pose_inliers = cv::recoverPose(essential_matrix, keyframe_points_undistorted, curr_points_undistorted, rotation, translation, 1.0, cv::Point2d(0.0, 0.0), inlier_mask);
                        double pose_inlier_ratio = static_cast<double>(pose_inliers) / static_cast<double>(inlier_count);
                        RCLCPP_INFO(this->get_logger(), "Pose quality: %d / %d = %.1f%%", pose_inliers, inlier_count, pose_inlier_ratio * 100);                            

                        bool pose_valid = pose_inliers >= MIN_POSE_INLIERS && pose_inlier_ratio >= MIN_POSE_INLIER_RATIO;
                        if(pose_valid) {
                            RCLCPP_INFO(this->get_logger(), "Pose accepted");
                            RCLCPP_INFO(this->get_logger(), "Pose inliers: %d", pose_inliers);
                            std::cout<< "Rotation:\n" << rotation << std::endl;
                            std::cout<< "Translation direction:\n" << translation << std::endl;

                            keyframe_keypoints_ = keypoints;
                            keyframe_descriptors_ = descriptors.clone();

                            RCLCPP_INFO(this->get_logger(), "New keyframe created");
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Pose rejected - keeping previous keyframe");
                        }
                    }
                }

                RCLCPP_INFO(this->get_logger(), "Features: %zu | KNN Candidates: %zu | Good Matches: %zu | Raw pairs: %zu | Undistorted pairs: %zu", keypoints.size(), knn_matches.size(), good_matches.size(), keyframe_points.size(), keyframe_points_undistorted.size());
                if(!keyframe_points_undistorted.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Raw prev: (%.2f, %.2f) | Undistorted: (%.4f, %.4f)", keyframe_points[0].x, keyframe_points[0].y, keyframe_points_undistorted[0].x, keyframe_points_undistorted[0].y);
                }
            }
        }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};

int main (int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<VisionProc>());

    rclcpp::shutdown();

    return 0;
}