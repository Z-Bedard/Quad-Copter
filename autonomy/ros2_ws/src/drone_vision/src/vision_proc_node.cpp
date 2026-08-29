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
        const int MIN_POSE_INLIERS = 25;
        const double MIN_POSE_INLIER_RATIO = 0.5;
        const double MIN_KEYFRAME_DISPLACEMENT_PX = 8.0;
        const double MAX_KEYFRAME_DISPLACEMENT_PX = 80.0;
        const double MAX_RELATIVE_ROTATION_DEG = 25.0;

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
                for(size_t i = 0; i <keyframe_points.size(); ++i) {
                    double dx = curr_points[i].x - keyframe_points[i].x;
                    double dy = curr_points[i].y - keyframe_points[i].y;
                    double displacement = std::sqrt(dx * dx + dy * dy);
                    displacements.push_back(displacement);
                }
                
                std::sort(displacements.begin(), displacements.end());
                double median_displacement = displacements[displacements.size() / 2];
                // RCLCPP_INFO(this->get_logger(), "Median displacement from keyframe: %.2f px", median_displacement);
                if(median_displacement < MIN_KEYFRAME_DISPLACEMENT_PX) {
                    // RCLCPP_INFO(this->get_logger(), "Keeping current keyframe");
                    return;
                }
                // if(median_displacement > MAX_KEYFRAME_DISPLACEMENT_PX) {
                //     RCLCPP_WARN(this->get_logger(), "Keyframe is stale (%.2f px) - resetting", median_displacement);
                //     if(keypoints.size() >= MIN_FEATURES_FOR_KEYFRAME) {
                //         keyframe_keypoints_ = keypoints;
                //         keyframe_descriptors_ = descriptors.clone();
                //         tracking_failure_count_ = 0;
                //         global_rotation_cw_ = cv::Mat::eye(3, 3, CV_64F);
                        
                //         RCLCPP_WARN(this->get_logger(), "Reference keyframe reset");
                //     } else {
                //         RCLCPP_WARN(this->get_logger(), "Current frame has too few features for reset: %zu", keypoints.size());
                //     }
                //     return;
                // }

                std::vector<cv::Point2f> keyframe_points_undistorted;
                std::vector<cv::Point2f> curr_points_undistorted;
                
                cv::undistortPoints(keyframe_points, keyframe_points_undistorted, camera_matrix_, distortion_coefficients_);
                cv::undistortPoints(curr_points, curr_points_undistorted, camera_matrix_, distortion_coefficients_);    

                if(keyframe_points_undistorted.size() >= 8){
                    cv::Mat inlier_mask;

                    cv::Mat essential_matrix = cv::findEssentialMat(keyframe_points_undistorted, curr_points_undistorted, 1.0, cv::Point2d(0.0, 0.0), cv::RANSAC, 0.999, 0.003, inlier_mask);
                    
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

                    cv::Mat correlation = cv::Mat::zeros(3, 3, CV_64F);
                    int ray_inliers = 0;

                    for(size_t i = 0; i < keyframe_rays.size(); ++i) {
                        if(inlier_mask.at<uchar>(static_cast<int>(i)) == 0) {
                            continue;
                        }
                        
                        cv::Mat keyframe_ray = (cv::Mat_<double>(3, 1) << keyframe_rays[i][0], keyframe_rays[i][1], keyframe_rays[i][2]);
                        cv::Mat curr_ray = (cv::Mat_<double>(3, 1) << curr_rays[i][0], curr_rays[i][1], curr_rays[i][2]);
                        correlation += curr_ray * keyframe_ray.t();

                        ray_inliers++;
                    }

                    if(ray_inliers >= 8) {
                        cv::SVD svd(correlation, cv::SVD::FULL_UV);
                        cv::Mat ray_rotation = svd.u * svd.vt;

                        if(cv::determinant(ray_rotation) < 0.0) {
                            cv::Mat correction = cv::Mat::eye(3, 3, CV_64F);
                            correction.at<double>(2, 2) = -1.0;
                            ray_rotation = svd.u * correction * svd.vt;
                        }

                        cv::Mat ray_rotation_vector;
                        cv::Rodrigues(ray_rotation, ray_rotation_vector);
                        double ray_rotation_deg = cv::norm(ray_rotation_vector) * 180.0 / CV_PI;

                        std::vector<double> ray_residuals_deg;
                        for(size_t i = 0; i < keyframe_rays.size(); ++i) {
                            cv::Mat keyframe_ray = (cv::Mat_<double>(3, 1) << keyframe_rays[i][0], keyframe_rays[i][1], keyframe_rays[i][2]);
                            cv::Mat predicted_ray = ray_rotation * keyframe_ray;
                            cv::Vec3d predicted(predicted_ray.at<double>(0, 0), predicted_ray.at<double>(1, 0), predicted_ray.at<double>(2, 0));
                            double dot = predicted.dot(curr_rays[i]);
                            dot = std::clamp(dot, -1.0, 1.0);
                            double residual_deg = std::acos(dot) * 180.0 / CV_PI;
                            ray_residuals_deg.push_back(residual_deg);
                        }

                        std::sort(ray_residuals_deg.begin(), ray_residuals_deg.end());
                        double median_ray_residual_deg = ray_residuals_deg[ray_residuals_deg.size() / 2];
                        RCLCPP_INFO(this->get_logger(), "Ray rotation: %.2f deg | Rays: %d | Median residual: %.2f deg", ray_rotation_deg, ray_inliers, median_ray_residual_deg);
                    }

                    if(!essential_matrix.empty()) {
                        int inlier_count = cv::countNonZero(inlier_mask);
                        // RCLCPP_INFO(this->get_logger(), "RANSAC inliers: %d / %zu", inlier_count, keyframe_points_undistorted.size());
                        if(inlier_count == 0) {
                            RCLCPP_WARN(this->get_logger(), "RANSAC returned zero inliers");
                            return;
                        }

                        cv::Mat rotation;
                        cv::Mat translation;

                        int pose_inliers = cv::recoverPose(essential_matrix, keyframe_points_undistorted, curr_points_undistorted, rotation, translation, 1.0, cv::Point2d(0.0, 0.0), inlier_mask);
                        double pose_inlier_ratio = static_cast<double>(pose_inliers) / static_cast<double>(inlier_count);
                        RCLCPP_INFO(this->get_logger(), "Pose quality: %d / %d = %.1f%%", pose_inliers, inlier_count, pose_inlier_ratio * 100);                            

                        cv::Mat relative_rotation_vector;
                        cv::Rodrigues(rotation, relative_rotation_vector);

                        double relative_rotation_rad = cv::norm(relative_rotation_vector);
                        double relative_rotation_deg = relative_rotation_rad * 180.0/CV_PI;
                        RCLCPP_INFO(this->get_logger(), "Relative rotation: %.2f deg", relative_rotation_deg);

                        bool pose_valid = pose_inliers >= MIN_POSE_INLIERS && pose_inlier_ratio >= MIN_POSE_INLIER_RATIO && relative_rotation_deg <= MAX_RELATIVE_ROTATION_DEG;
                        if(pose_valid) {
                            RCLCPP_INFO(this->get_logger(), "Pose accepted");
                            global_rotation_cw_ = rotation * global_rotation_cw_;
                            // std::cout << "Global world -> camera rotation: \n" << global_rotation_cw_ << std::endl;
                            
                            cv::Mat camera_rotation_world = global_rotation_cw_.t();
                            cv::Mat rotation_cb = rotation_bc_.t();
                            cv::Mat body_rotation_world = rotation_bc_ * camera_rotation_world * rotation_cb;
                            
                            cv::Mat rotation_vector;
                            cv::Rodrigues(body_rotation_world, rotation_vector);
                            cv::Mat rotation_vector_deg = rotation_vector * (180.0/CV_PI);
                            std::cout <<"Body rotation vector [deg]: \n" << rotation_vector_deg << std::endl;

                            // RCLCPP_INFO(this->get_logger(), "Pose inliers: %d", pose_inliers);
                            // std::cout<< "Rotation:\n" << rotation << std::endl;
                            // std::cout<< "Translation direction:\n" << translation << std::endl;

                            keyframe_keypoints_ = keypoints;
                            keyframe_descriptors_ = descriptors.clone();

                            RCLCPP_INFO(this->get_logger(), "New keyframe created");
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Pose rejected - keeping previous keyframe");
                            if(relative_rotation_deg > MAX_RELATIVE_ROTATION_DEG) {
                                RCLCPP_WARN(this->get_logger(), "Pose rejected - relative rotation: %.2f deg", relative_rotation_deg);
                            }

                            cv::Mat homography_mask;
                            cv::Mat homography = cv::findHomography(keyframe_points_undistorted, curr_points_undistorted, cv::RANSAC, 0.003, homography_mask);

                            if(!homography.empty()) {
                                int homography_inliers = cv::countNonZero(homography_mask);

                                double homography_inlier_ratio = static_cast<double>(homography_inliers) / static_cast<double>(keyframe_points_undistorted.size());

                                RCLCPP_INFO(this->get_logger(), "Pose rejected | Homography: %d / %zu = %.1f%%", homography_inliers, keyframe_points_undistorted.size(), homography_inlier_ratio * 100.0);
                            }
                        }
                    }
                }

                // RCLCPP_INFO(this->get_logger(), "Features: %zu | KNN Candidates: %zu | Good Matches: %zu | Raw pairs: %zu | Undistorted pairs: %zu", keypoints.size(), knn_matches.size(), good_matches.size(), keyframe_points.size(), keyframe_points_undistorted.size());
                // if(!keyframe_points_undistorted.empty()) {
                //     RCLCPP_INFO(this->get_logger(), "Raw prev: (%.2f, %.2f) | Undistorted: (%.4f, %.4f)", keyframe_points[0].x, keyframe_points[0].y, keyframe_points_undistorted[0].x, keyframe_points_undistorted[0].y);
                // }
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