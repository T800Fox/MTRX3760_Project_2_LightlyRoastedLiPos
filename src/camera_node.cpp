#include <camera_node.hpp>
using namespace mtrx3760_lrl_warehousebot::msg;

// ============================================================================
// Base Camera Class Implementation
// ============================================================================

Camera::Camera(const std::string& node_name) : Node(node_name)
{
    // Initialize publisher
    marker_pub_ = create_publisher<DetectedMarker>("/detections/raw", 10);
    
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

ArucoCamera::ArucoCamera() : Camera("oogway_camera_node")
{
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
            
            // Draw distance vector arrow
            cv::Point2f arrow_end = center + cv::Point2f(translation[0] * 100, translation[1] * 100);
            cv::arrowedLine(frame, center, arrow_end, cv::Scalar(0, 255, 0), 2);
            
            // Draw distance text
            std::string distance_text = "ID:" + std::to_string(ids[i]) + " D:" + 
                                      std::to_string(distance).substr(0, 4) + "m";
            cv::putText(frame, distance_text, center + cv::Point2f(10, -10), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            
            // Draw position coordinates (x,y,z) with more precision
            char pos_buffer[100];
            snprintf(pos_buffer, sizeof(pos_buffer), "Pos: (%.3f, %.3f, %.3f)", 
                    translation[0], translation[1], translation[2]);
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
            
            // Publish marker detection
            auto detection_msg = DetectedMarker();
            detection_msg.header.stamp = this->now();
            detection_msg.header.frame_id = "camera_frame";
            detection_msg.id = ids[i];
            detection_msg.position.x = translation[0];
            detection_msg.position.y = translation[1];
            detection_msg.position.z = translation[2];
            detection_msg.confidence = corner_confidence;
            
            marker_pub_->publish(detection_msg);
            
            // Draw coordinate axes
            cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], tag_size * 0.5);
        }
    }
    
    cv::imshow("Frame", frame);
    cv::waitKey(1);
}

// Spin
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoCamera>());
    rclcpp::shutdown();
    
    return 0;
}