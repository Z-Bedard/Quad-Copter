#include <algorithm>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class VisionProc : public rclcpp::Node {
    public:
        VisionProc() : Node("vision_proc_node") {
            image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&VisionProc::image_callback, this, std::placeholders::_1));
            orientation_publisher_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/drone/vision/orientation", 10);

            matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false);

            orb_ = cv::ORB::create(1000);
        }

    private:
        const int MIN_POSE_INLIERS = 25;
        const double MIN_POSE_INLIER_RATIO = 0.5;
        const double MIN_KEYFRAME_DISPLACEMENT_PX = 8.0;
        const double MAX_RELATIVE_ROTATION_DEG = 25.0;
        const double MAX_RAY_RESIDUAL_DEG = 3.0;
        const double MIN_RAY_INLIER_RATIO = 0.6;

        const int MAX_TRACKING_FAILURES = 3;
        const int RAY_RANSAC_ITERATIONS = 50;
        const size_t MIN_RAY_INLIERS = 8;
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
        cv::Mat global_rotation_cw_ = cv::Mat::eye(3, 3, CV_64F);

        cv::Mat rotation_bc_ = (cv::Mat_<double>(3, 3) <<
            0.0,  0.0,  1.0,
            -1.0,  0.0,  0.0,
            0.0, -1.0,  0.0);

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

                for(const auto &match_pair : knn_matches) {
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
                    if(tracking_failure_count_ < MAX_TRACKING_FAILURES) {
                        tracking_failure_count_++;
                    }

                    RCLCPP_WARN(this->get_logger(), "Tracking weak: %zu good matches | Failure %d / %d", good_matches.size(), tracking_failure_count_, MAX_TRACKING_FAILURES);

                    if(tracking_failure_count_ >= MAX_TRACKING_FAILURES) {
                        if(keypoints.size() >= MIN_FEATURES_FOR_KEYFRAME) {
                            tracking_failure_count_ = 0;
                            keyframe_keypoints_ = keypoints;
                            keyframe_descriptors_ = descriptors.clone();
                            global_rotation_cw_ = cv::Mat::eye(3, 3, CV_64F);

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

                for(size_t i = 0; i < keyframe_points.size(); ++i) {
                    double dx = curr_points[i].x - keyframe_points[i].x;
                    double dy = curr_points[i].y - keyframe_points[i].y;
                    double displacement = std::sqrt(dx * dx + dy * dy);

                    displacements.push_back(displacement);
                }
                
                std::sort(displacements.begin(), displacements.end());

                double median_displacement = displacements[displacements.size() / 2];

                if(median_displacement < MIN_KEYFRAME_DISPLACEMENT_PX) {
                    return;
                }

                std::vector<cv::Point2f> keyframe_points_undistorted;
                std::vector<cv::Point2f> curr_points_undistorted;
                
                cv::undistortPoints(keyframe_points, keyframe_points_undistorted, camera_matrix_, distortion_coefficients_);
                cv::undistortPoints(curr_points, curr_points_undistorted, camera_matrix_, distortion_coefficients_);    

                if(keyframe_points_undistorted.size() >= 8) {
                    std::vector<cv::Vec3d> keyframe_rays;
                    std::vector<cv::Vec3d> curr_rays;

                    for(size_t i = 0; i < keyframe_points_undistorted.size(); ++i) {
                        cv::Vec3d keyframe_ray(keyframe_points_undistorted[i].x, keyframe_points_undistorted[i].y, 1.0);
                        cv::Vec3d curr_ray(curr_points_undistorted[i].x, curr_points_undistorted[i].y, 1.0);
                    
                        keyframe_ray /= cv::norm(keyframe_ray);
                        curr_ray /= cv::norm(curr_ray);

                        keyframe_rays.push_back(keyframe_ray);
                        curr_rays.push_back(curr_ray);
                    }

                    std::vector<int> best_ray_inliers;
                    cv::RNG rng(static_cast<uint64>(cv::getTickCount()));

                    for(int iteration = 0; iteration < RAY_RANSAC_ITERATIONS; ++iteration) {
                        int index_a = rng.uniform(0, static_cast<int>(keyframe_rays.size()));
                        
                        int index_b;

                        do {
                            index_b = rng.uniform(0, static_cast<int>(keyframe_rays.size()));
                        } while(index_b == index_a);

                        int index_c;

                        do {
                            index_c = rng.uniform(0, static_cast<int>(keyframe_rays.size()));
                        } while(index_c == index_a || index_c == index_b);

                        cv::Mat sample_correlation = cv::Mat::zeros(3, 3, CV_64F);
                        int sample_indices[3] = {index_a, index_b, index_c};

                        for(int sample_index : sample_indices) {
                            cv::Mat keyframe_ray = (cv::Mat_<double>(3, 1) << keyframe_rays[sample_index][0], keyframe_rays[sample_index][1], keyframe_rays[sample_index][2]);
                            cv::Mat curr_ray = (cv::Mat_<double>(3, 1) << curr_rays[sample_index][0], curr_rays[sample_index][1], curr_rays[sample_index][2]);
                            sample_correlation += curr_ray * keyframe_ray.t();
                        }

                        cv::SVD sample_svd(sample_correlation, cv::SVD::FULL_UV);
                        cv::Mat candidate_rotation = sample_svd.u * sample_svd.vt;

                        if(cv::determinant(candidate_rotation) < 0.0) {
                            cv::Mat correction = cv::Mat::eye(3, 3, CV_64F);
                            correction.at<double>(2, 2) = -1.0;

                            candidate_rotation = sample_svd.u * correction * sample_svd.vt;
                        }

                        std::vector<int> candidate_inliers;

                        for(size_t i = 0; i < keyframe_rays.size(); ++i) {                            
                            cv::Mat keyframe_ray = (cv::Mat_<double>(3, 1) << keyframe_rays[i][0], keyframe_rays[i][1], keyframe_rays[i][2]);
                            cv::Mat predicted_ray = candidate_rotation * keyframe_ray;
                            cv::Vec3d predicted(predicted_ray.at<double>(0, 0), predicted_ray.at<double>(1, 0), predicted_ray.at<double>(2, 0));
                            double dot = predicted.dot(curr_rays[i]);
                            dot = std::clamp(dot, -1.0, 1.0);

                            double residual_deg = std::acos(dot) * 180.0 / CV_PI;

                            if(residual_deg <= MAX_RAY_RESIDUAL_DEG) {
                                candidate_inliers.push_back(static_cast<int>(i));
                            }
                        }

                        if(candidate_inliers.size() > best_ray_inliers.size()) {
                            best_ray_inliers = candidate_inliers;
                        }
                    }
                    
                    cv::Mat final_ray_rotation;
                    bool ray_pose_valid = false;
                    double final_ray_rotation_deg = 0.0;                    
                    double ray_inlier_ratio = 0.0;

                    if(best_ray_inliers.size() >= MIN_RAY_INLIERS) {
                        cv::Mat final_correlation = cv::Mat::zeros(3, 3, CV_64F);

                        for(int index : best_ray_inliers) {
                            cv::Mat keyframe_ray = (cv::Mat_<double>(3, 1) << keyframe_rays[index][0], keyframe_rays[index][1], keyframe_rays[index][2]);
                            cv::Mat curr_ray = (cv::Mat_<double>(3, 1) << curr_rays[index][0], curr_rays[index][1], curr_rays[index][2]);
                            final_correlation += curr_ray * keyframe_ray.t();
                        }

                        cv::SVD final_svd(final_correlation, cv::SVD::FULL_UV);
                        final_ray_rotation = final_svd.u * final_svd.vt;

                        if(cv::determinant(final_ray_rotation) < 0.0) {
                            cv::Mat correction = cv::Mat::eye(3, 3, CV_64F);
                            correction.at<double>(2, 2) = -1.0;

                            final_ray_rotation = final_svd.u * correction * final_svd.vt;
                        }

                        std::vector<int> final_ray_inliers;
                        std::vector<double> final_ray_residuals_deg;

                        for(size_t index = 0; index < keyframe_rays.size(); ++index) {
                            cv::Mat keyframe_ray = (cv::Mat_<double>(3, 1) << keyframe_rays[index][0], keyframe_rays[index][1], keyframe_rays[index][2]);
                            cv::Mat predicted_ray = final_ray_rotation * keyframe_ray;
                            cv::Vec3d predicted(predicted_ray.at<double>(0, 0), predicted_ray.at<double>(1, 0), predicted_ray.at<double>(2, 0));
                            double dot = predicted.dot(curr_rays[index]);
                            dot = std::clamp(dot, -1.0, 1.0);

                            double residual_deg = std::acos(dot) * 180.0 / CV_PI;  

                            if(residual_deg <= MAX_RAY_RESIDUAL_DEG){
                                final_ray_inliers.push_back(static_cast<int>(index));
                                final_ray_residuals_deg.push_back(residual_deg);
                            }
                        }

                        ray_inlier_ratio = static_cast<double>(final_ray_inliers.size()) / static_cast<double>(keyframe_rays.size());
                        double final_median_ray_residual_deg = 0.0;
                        if(!final_ray_residuals_deg.empty()){
                            std::sort(final_ray_residuals_deg.begin(), final_ray_residuals_deg.end());
                            final_median_ray_residual_deg = final_ray_residuals_deg[final_ray_residuals_deg.size() / 2];
                        }
                        cv::Mat final_ray_rotation_vector;
                        cv::Rodrigues(final_ray_rotation, final_ray_rotation_vector);
                        final_ray_rotation_deg = cv::norm(final_ray_rotation_vector) * 180.0 / CV_PI;

                        ray_pose_valid = final_ray_inliers.size() >= MIN_RAY_INLIERS && ray_inlier_ratio >= MIN_RAY_INLIER_RATIO && final_ray_rotation_deg <= MAX_RELATIVE_ROTATION_DEG;
                        RCLCPP_INFO(this->get_logger(), "Ray RANSAC: %.2f deg | Inliers: %zu / %zu = %.1f%% | Median residual: %.2f deg", final_ray_rotation_deg, final_ray_inliers.size(), keyframe_rays.size(), ray_inlier_ratio * 100.0, final_median_ray_residual_deg);
                    }

                    cv::Mat selected_rotation;
                    bool rotation_valid = false;

                    cv::Mat inlier_mask;

                    cv::Mat essential_matrix = cv::findEssentialMat(keyframe_points_undistorted, curr_points_undistorted, 1.0, cv::Point2d(0.0, 0.0), cv::RANSAC, 0.999, 0.003, inlier_mask);

                    if(!essential_matrix.empty()) {
                        int inlier_count = cv::countNonZero(inlier_mask);

                        if(inlier_count > 0) {
                            cv::Mat rotation;
                            cv::Mat translation;

                            int pose_inliers = cv::recoverPose(essential_matrix, keyframe_points_undistorted, curr_points_undistorted, rotation, translation, 1.0, cv::Point2d(0.0, 0.0), inlier_mask);
                            double pose_inlier_ratio = static_cast<double>(pose_inliers) / static_cast<double>(inlier_count);

                            cv::Mat relative_rotation_vector;
                            cv::Rodrigues(rotation, relative_rotation_vector);

                            double relative_rotation_deg = cv::norm(relative_rotation_vector) * 180.0 / CV_PI;

                            RCLCPP_INFO(this->get_logger(), "Pose quality: %d / %d = %.1f%%", pose_inliers, inlier_count, pose_inlier_ratio * 100.0);
                            RCLCPP_INFO(this->get_logger(), "Relative rotation: %.2f deg", relative_rotation_deg);
                            bool pose_valid = pose_inliers >= MIN_POSE_INLIERS && pose_inlier_ratio >= MIN_POSE_INLIER_RATIO && relative_rotation_deg <= MAX_RELATIVE_ROTATION_DEG;

                            if(pose_valid) {
                                selected_rotation = rotation;
                                rotation_valid = true;

                                RCLCPP_INFO(this->get_logger(), "Rotation accepted - recoverPose");
                            }
                        }
                    }

                    if(!rotation_valid && ray_pose_valid) {
                        selected_rotation = final_ray_rotation;
                        rotation_valid = true;

                        RCLCPP_INFO(this->get_logger(), "Rotation accepted - Ray RANSAC");
                    }

                    if(rotation_valid) {
                        global_rotation_cw_ = selected_rotation * global_rotation_cw_;
                            
                        cv::Mat camera_rotation_world = global_rotation_cw_.t();
                        cv::Mat rotation_cb = rotation_bc_.t();
                        cv::Mat body_rotation_world = rotation_bc_ * camera_rotation_world * rotation_cb;
                            
                        cv::Mat rotation_vector;
                        cv::Rodrigues(body_rotation_world, rotation_vector);

                        cv::Mat rotation_vector_deg = rotation_vector * (180.0 / CV_PI);
                        std::cout << "Body rotation vector [deg]: \n" << rotation_vector_deg << std::endl;

                        tf2::Matrix3x3 tf_rotation(
                            body_rotation_world.at<double>(0, 0),
                            body_rotation_world.at<double>(0, 1),
                            body_rotation_world.at<double>(0, 2),

                            body_rotation_world.at<double>(1, 0),
                            body_rotation_world.at<double>(1, 1),
                            body_rotation_world.at<double>(1, 2),

                            body_rotation_world.at<double>(2, 0),
                            body_rotation_world.at<double>(2, 1),
                            body_rotation_world.at<double>(2, 2));
                        
                        tf2::Quaternion quaternion;
                        tf_rotation.getRotation(quaternion);
                        quaternion.normalize();

                        geometry_msgs::msg::Quaternion orientation_msg;

                        orientation_msg.x = quaternion.x();
                        orientation_msg.y = quaternion.y();
                        orientation_msg.z = quaternion.z();
                        orientation_msg.w = quaternion.w();

                        orientation_publiser_->publish(orientation_msg);

                        keyframe_keypoints_ = keypoints;
                        keyframe_descriptors_ = descriptors.clone();

                        RCLCPP_INFO(this->get_logger(), "New keyframe created");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "No valid rotation estimate - keeping previous keyframe");
                    }
                }
            }
        }

        rclcpp::Publisher<geometry_msgs::msg::Quaternion>SharedPtr orientation_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<VisionProc>());

    rclcpp::shutdown();

    return 0;
}