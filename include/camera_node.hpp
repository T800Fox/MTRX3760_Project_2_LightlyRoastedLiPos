#ifndef MARKER_TRACKING_CAMERA_NODE_HPP_
#define MARKER_TRACKING_CAMERA_NODE_HPP_

#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <mtrx3760_lrl_warehousebot/msg/detected_marker.hpp>

using DetectedMarker = mtrx3760_lrl_warehousebot::msg::DetectedMarker;

// Base Camera class for general camera operations
class Camera : public rclcpp::Node
{
    public:
        Camera(const std::string& node_name);
        virtual ~Camera();
        
    protected:
        // ROS topic publisher
        rclcpp::Publisher<DetectedMarker>::SharedPtr marker_pub_;
        
        // Virtual function to be overridden by derived classes
        virtual void processImage(const cv::Mat& frame) = 0;
        
    private:
        // ROS topic subscriber
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_sub_;
        
        // Subscription callback
        void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image);
};

// ArUco Camera class - derived from Camera
class ArucoCamera : public Camera
{
    public:
        ArucoCamera();
        ~ArucoCamera();
        
    private:
        // Override processImage to handle ArUco detection
        void processImage(const cv::Mat& frame) override;

        // Declare members
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        
        // Camera calibration parameters (you may need to adjust these for your camera)
        cv::Mat camera_matrix;
        cv::Mat dist_coeffs;
        
        // Tag size in meters (adjust this based on your actual ArUco tag size)
        double tag_size;
};

#endif