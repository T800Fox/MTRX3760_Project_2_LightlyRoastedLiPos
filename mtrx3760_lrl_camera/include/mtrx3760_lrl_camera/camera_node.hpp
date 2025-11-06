// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: camera_node.hpp
// Author(s): Kyle Soepono
//
// Header file for camera node classes. Defines base Camera class and ArucoCamera
// derived class for ArUco marker detection and tracking. Uses OpenCV for marker
// detection and TF2 for coordinate transformations.

#ifndef MARKER_TRACKING_CAMERA_NODE_HPP_
#define MARKER_TRACKING_CAMERA_NODE_HPP_

#include <memory>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "mtrx3760_lrl_interfaces/msg/marker_detection.hpp"

using MarkerDetection = mtrx3760_lrl_interfaces::msg::MarkerDetection;

// Running statistics per marker ID (incremental mean and variance)
// Uses Welford's algorithm for online variance calculation
struct RunningMarkerStats {
    int32_t count = 0;
    double mean_x = 0.0;
    double mean_y = 0.0;
    double mean_z = 0.0;
    double m2_x = 0.0; // Sum of squared deltas for variance (Welford's algorithm)
    double m2_y = 0.0;
    double m2_z = 0.0;
    double mean_confidence = 0.0;
};

// ============================================================================
// Base Camera Class
// ============================================================================
// Abstract base class for camera operations. Provides common functionality
// for image subscription and publishing. Derived classes implement specific
// marker detection algorithms.
class Camera : public rclcpp::Node
{
    public:
        Camera(const std::string& node_name);
        virtual ~Camera();
        
    protected:
        // ROS topic publisher for marker detections
        rclcpp::Publisher<MarkerDetection>::SharedPtr marker_pub_;
        
        // Pure virtual function to be overridden by derived classes
        // Processes incoming image frames for marker detection
        virtual void processImage(const cv::Mat& frame) = 0;
        
    private:
        // ROS topic subscriber for compressed camera images
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_sub_;
        
        // Callback function for incoming compressed images
        // Decompresses image and calls processImage()
        void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image);
};

// ============================================================================
// ArucoCamera Class
// ============================================================================
// Derived class implementing ArUco marker detection and tracking.
// Detects ArUco markers, estimates their pose, transforms to global coordinates
// using TF2, and publishes MarkerDetection messages with running statistics.
class ArucoCamera : public Camera
{
    public:
        ArucoCamera();
        ~ArucoCamera();
        
    private:
        // Override processImage to handle ArUco marker detection
        void processImage(const cv::Mat& frame) override;
        
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        
        // Map of marker ID to running statistics (mean, variance, count)
        std::map<int32_t, RunningMarkerStats> marker_stats_;
        std::map<int32_t, cv::Mat> closest_images_;
        std::map<int32_t, double> closest_distances_;
        
        double confidence_threshold_;
        int max_observations_per_id_;  // Deprecated: no longer used

        // Process detected marker: filter by confidence, transform to global coords,
        // and update statistics
        void process_marker_detection(int32_t id, double x, double y, double z, double confidence);
        
        // Add observation to running statistics using Welford's algorithm
        void add_observation(int32_t id, double x, double y, double z, double confidence);

        // ArUco dictionary (4x4 markers with 50 possible IDs)
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        
        // Camera calibration matrix (3x3 intrinsic parameters)
        // Contains focal length (fx, fy) and principal point (cx, cy)
        cv::Mat camera_matrix;
        
        // Distortion coefficients (4x1 vector)
        cv::Mat dist_coeffs;
        
        // Physical size of ArUco markers in meters
        double tag_size;
        
};

#endif