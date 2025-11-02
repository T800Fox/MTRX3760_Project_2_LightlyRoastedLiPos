#include <cmath>
#include <tf2/exceptions.h>
#include <sensor_msgs/image_encodings.hpp>

#include "mtrx3760_lrl_camera/camera_node.hpp"

using namespace mtrx3760_lrl_interfaces::msg;

// ============================================================================
// Base Camera Class Implementation
// ============================================================================

Camera::Camera(const std::string& node_name) : Node(node_name)
{
    // Initialize publisher
    marker_pub_ = create_publisher<MarkerDetection>("/detections/raw", 10);
    
    // Initialize subscribers
    // Use compressed images (better for bandwidth)
    camera_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>("/camera/image_raw/compressed", 10, 
        std::bind(&Camera::camera_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Base Camera node initialized: %s", node_name.c_str());
}

Camera::~Camera()
{
    RCLCPP_INFO(this->get_logger(), "Base Camera node terminated");
}

void Camera::camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image)
{
    cv::Mat frame;
    try {
        // Decompress compressed image
        frame = cv::imdecode(cv::Mat(image->data), cv::IMREAD_COLOR);
        if (frame.empty()) {
            return;
        }
    } catch (const cv::Exception & e) {
        return;
    }
    
    // Call virtual function to process the image
    processImage(frame);
}

// ============================================================================
// ArucoCamera Class Implementation
// ============================================================================

ArucoCamera::ArucoCamera() : Camera("lrl_camera_node")
{
    // Declare parameters (for controller functionality)
    this->declare_parameter("confidence_threshold", 0.7);
    this->declare_parameter("max_observations_per_id", 10);
    
    confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
    max_observations_per_id_ = this->get_parameter("max_observations_per_id").as_int();
    
    // Initialize TF2 buffer and listener (for coordinate transforms)
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Init members
    // Define the ArUco dictionary to use (4x4)
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    
    // Set tag size in meters (adjust this based on your actual ArUco tag size)
    tag_size = 0.05; // 5cm tags
    
    // Initialize camera matrix for Raspberry Pi Camera Module v2 at 640x480 resolution
    // Based on sensor cropping at 640x480, effective focal length is ~1043 pixels
    // Principal point is at image center (320, 240)

    // Correction factor: if distance is 5% too high, reduce focal length by 5%
    double focal_length_corrected = 529.0 * 0.95;  // 5% reduction

    camera_matrix = (cv::Mat_<double>(3,3) << 
        focal_length_corrected, 0, 320.0,    // fx, 0, cx (effective focal length due to cropping)
        0, focal_length_corrected, 240.0,    // 0, fy, cy (effective focal length due to cropping)
        0, 0, 1);                             // 0, 0, 1
    
    // Initialize distortion coefficients (usually all zeros for simulation cameras)
    dist_coeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    
    RCLCPP_INFO(this->get_logger(), "Oogway Goal Checker simulation node has been created!!!!!");
    RCLCPP_INFO(this->get_logger(), "Using tag size: %.3f meters", tag_size);
    RCLCPP_INFO(this->get_logger(), "Camera matrix focal length: fx=%.2f, fy=%.2f", 
                camera_matrix.at<double>(0,0), camera_matrix.at<double>(1,1));
    RCLCPP_INFO(this->get_logger(), "Confidence threshold: %.2f", confidence_threshold_);
    RCLCPP_INFO(this->get_logger(), "Max observations per ID (deprecated): %d", max_observations_per_id_);
}

ArucoCamera::~ArucoCamera()
{
    RCLCPP_INFO(this->get_logger(), "Oogway Goal Checker simulation node has been terminated");
}

void ArucoCamera::processImage(const cv::Mat& frame)
{
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);
    
    //Draw tags and calculate distances
    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(frame, corners, ids);
        
        // Calculate pose using fixed camera parameters
        // Calculate pose for each detected marker
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, tag_size, camera_matrix, dist_coeffs, rvecs, tvecs);

        // Publish and draw distance vectors for each detected marker
        for(size_t i = 0; i < ids.size(); i++) {
            // Get the center of the marker
            cv::Point2f center(0, 0);
            for(int j = 0; j < 4; j++) {
                center += corners[i][j];
            }
            center /= 4.0;
            
            // Get the translation vector (distance from camera to marker)
            cv::Vec3d translation = tvecs[i];
            double distance = sqrt(translation[0]*translation[0] + 
                                 translation[1]*translation[1] + 
                                 translation[2]*translation[2]);
            
            // Track closest image: save first image or update if current is closer
            bool is_closer = false;
            auto dist_it = closest_distances_.find(ids[i]);
            if (dist_it == closest_distances_.end()) {
                // First observation for this marker ID
                is_closer = true;
                closest_distances_[ids[i]] = distance;
                frame.copyTo(closest_images_[ids[i]]);
                RCLCPP_INFO(this->get_logger(), "First detection of marker ID %d at distance %.3fm", ids[i], distance);
            } else if (distance < dist_it->second) {
                // Current observation is closer than previous closest
                is_closer = true;
                closest_distances_[ids[i]] = distance;
                frame.copyTo(closest_images_[ids[i]]);
                RCLCPP_INFO(this->get_logger(), "Closer image for marker ID %d: %.3fm (was %.3fm)", 
                            ids[i], distance, dist_it->second);
            }
            
            // Draw distance vector arrow
            cv::Point2f arrow_end = center + cv::Point2f(translation[0] * 100, translation[1] * 100);
            cv::arrowedLine(frame, center, arrow_end, cv::Scalar(0, 255, 0), 2);
            
            // Draw distance text
            std::string distance_text = "ID:" + std::to_string(ids[i]) + " D:" + 
                                      std::to_string(distance).substr(0, 4) + "m";
            cv::putText(frame, distance_text, center + cv::Point2f(10, -10), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            
            // Draw position coordinates (remapped convention: x=right, y=away, z=up)
            char pos_buffer[100];
            double remapped_x = translation[0];          // right (+)
            double remapped_y = translation[2];          // away (+)
            double remapped_z = -translation[1];         // up (+)
            snprintf(pos_buffer, sizeof(pos_buffer), "Pos: (%.3f, %.3f, %.3f)", 
                    remapped_x, remapped_y, remapped_z);
            cv::putText(frame, pos_buffer, center + cv::Point2f(10, 5), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
            
            // Draw rotation angles (roll, pitch, yaw) with more precision
            cv::Vec3d rotation = rvecs[i];
            char rot_buffer[100];
            snprintf(rot_buffer, sizeof(rot_buffer), "Rot: (%.3f, %.3f, %.3f)", 
                    rotation[0], rotation[1], rotation[2]);
            cv::putText(frame, rot_buffer, center + cv::Point2f(10, 20), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
            
            // Calculate corner detection confidence
            // Check if corners form a good rectangle (circularity ratio)
            std::vector<cv::Point2f> corners_vec = corners[i];
            double perimeter = cv::arcLength(corners_vec, true);
            double area_contour = cv::contourArea(corners_vec);
            double corner_confidence = (perimeter > 0) ? (4 * M_PI * area_contour) / (perimeter * perimeter) : 0;
            
            // Draw confidence text
            std::string conf_text = "Conf: " + std::to_string(corner_confidence).substr(0, 4);
            cv::Scalar conf_color = (corner_confidence > 0.7) ? cv::Scalar(0, 255, 0) : 
                                   (corner_confidence > 0.4) ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255);
            cv::putText(frame, conf_text, center + cv::Point2f(10, 35), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, conf_color, 1);
            
            // Process marker through odometry fusion using remapped convention (x=right, y=away, z=up)
            // This updates the stats needed for global position, covariance, and observation count
            process_marker_detection(ids[i], translation[0], translation[2], -translation[1], corner_confidence);
            
            // Get stats for this marker ID and publish every frame
            auto stats_it = marker_stats_.find(ids[i]);
            if (stats_it != marker_stats_.end() && stats_it->second.count > 0) {
                const auto & stats = stats_it->second;
                
                // Calculate covariance_radius
                double var_x = (stats.count > 1) ? (stats.m2_x / static_cast<double>(stats.count - 1)) : 0.0;
                double var_y = (stats.count > 1) ? (stats.m2_y / static_cast<double>(stats.count - 1)) : 0.0;
                double var_z = (stats.count > 1) ? (stats.m2_z / static_cast<double>(stats.count - 1)) : 0.0;
                double covariance_radius = std::sqrt(std::max(0.0, var_x + var_y + var_z));
                
                // Publish MarkerDetection message with updated stats
                MarkerDetection pos_msg;
                pos_msg.id = ids[i];
                pos_msg.global_position.x = stats.mean_x;
                pos_msg.global_position.y = stats.mean_y;
                pos_msg.global_position.z = stats.mean_z;
                pos_msg.confidence = stats.mean_confidence;
                pos_msg.covariance_radius = covariance_radius;
                pos_msg.observation_count = stats.count;
                
                // Only include image if this is a closer observation (saves bandwidth)
                if (is_closer) {
                    auto img_it = closest_images_.find(ids[i]);
                    if (img_it != closest_images_.end() && !img_it->second.empty()) {
                        try {
                            std_msgs::msg::Header img_header;
                            img_header.stamp = this->now();
                            img_header.frame_id = "camera_frame";
                            auto img_bridge = cv_bridge::CvImage(img_header, sensor_msgs::image_encodings::BGR8, img_it->second);
                            pos_msg.closest_image = *img_bridge.toImageMsg();
                            RCLCPP_INFO(this->get_logger(), "Updated closest image for ID %d (distance: %.3fm)", 
                                        ids[i], distance);
                        } catch (const cv_bridge::Exception& e) {
                            RCLCPP_WARN(this->get_logger(), "Failed to convert closest image for ID %d: %s", ids[i], e.what());
                        }
                    }
                }
                // Otherwise, leave closest_image empty (default constructed sensor_msgs::Image)
                
                marker_pub_->publish(pos_msg);
            }
            
            // Draw coordinate axes
            cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], tag_size * 0.5);
        }
    }
    
    cv::imshow("Frame", frame);
    cv::waitKey(1);
}

// ============================================================================
// Controller Functionality (merged from controller_node)
// ============================================================================

void ArucoCamera::process_marker_detection(int32_t id, double x, double y, double z, double confidence)
{
    // Filter by confidence (green = >0.7)
    if (confidence < confidence_threshold_) {
        return;
    }
    
    try {
        // Get base_link position and rotation in odom frame from TF2
        auto transform = tf_buffer_->lookupTransform(
            // "odom", "base_link", this->now(), rclcpp::Duration::from_seconds(0.1));
            "odom", "base_link", tf2::TimePointZero);
        
        // Extract robot position in odom frame (x, y, z)
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
        double robot_z = transform.transform.translation.z;
        
        // Extract yaw from quaternion rotation
        auto& q = transform.transform.rotation;
        double robot_yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        
        // Desired convention at input:
        //   x: right positive
        //   y: away/forward positive
        //   z: up positive
        // Map to base_link frame (x=forward, y=left, z=up):
        //   base x (forward) = y (away)
        //   base y (left)    = -x (right)
        //   base z (up)      = z (up)
        double local_x = y;      // away → forward
        double local_y = -x;     // right → -left
        double local_z = z;      // up → up
        
        // Rotate the x,y components by robot yaw in 2D plane
        double cos_yaw = cos(robot_yaw);
        double sin_yaw = sin(robot_yaw);
        
        // Rotate local vector (x,y) and add to robot position
        double global_x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
        double global_y = robot_y + local_x * sin_yaw + local_y * cos_yaw;
        double global_z = robot_z + local_z;  // Height just adds directly
        
        // Update running stats (per ID); publishing happens once per frame
        add_observation(id, global_x, global_y, global_z, confidence);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Could not transform from base_link to odom: %s", ex.what());
        return;
    }
}

void ArucoCamera::add_observation(int32_t id, double x, double y, double z, double confidence)
{
    auto & stats = marker_stats_[id];
    
    // Incremental mean and variance (Welford's algorithm), unweighted
    stats.count += 1;
    
    // X
    double delta_x = x - stats.mean_x;
    stats.mean_x += delta_x / static_cast<double>(stats.count);
    double delta2_x = x - stats.mean_x;
    stats.m2_x += delta_x * delta2_x;
    
    // Y
    double delta_y = y - stats.mean_y;
    stats.mean_y += delta_y / static_cast<double>(stats.count);
    double delta2_y = y - stats.mean_y;
    stats.m2_y += delta_y * delta2_y;
    
    // Z
    double delta_z = z - stats.mean_z;
    stats.mean_z += delta_z / static_cast<double>(stats.count);
    double delta2_z = z - stats.mean_z;
    stats.m2_z += delta_z * delta2_z;
    
    // Confidence running average
    double delta_c = confidence - stats.mean_confidence;
    stats.mean_confidence += delta_c / static_cast<double>(stats.count);
}

// Removed eviction: we no longer keep full histories

// Spin
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoCamera>());
    rclcpp::shutdown();
    
    return 0;
}