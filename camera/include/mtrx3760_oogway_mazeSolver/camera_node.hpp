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
#include <marker_tracking/msg/detected_marker.hpp>
#include <marker_tracking/msg/marker_positions.hpp>

using DetectedMarker = marker_tracking::msg::DetectedMarker;
using MarkerPositions = marker_tracking::msg::MarkerPositions;
using MarkerPosition = marker_tracking::msg::MarkerPosition;

struct MarkerObservation {
    double x, y, z;
    double confidence;
    rclcpp::Time timestamp;
    
    MarkerObservation(double x_, double y_, double z_, double conf_, rclcpp::Time t)
        : x(x_), y(y_), z(z_), confidence(conf_), timestamp(t) {}
};

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
        
        // TF2 buffer and listener (for coordinate transforms)
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        
        // Marker positions publisher (for controller functionality)
        rclcpp::Publisher<MarkerPositions>::SharedPtr positions_pub_;
        
        // Storage: map of ID -> vector of observations
        std::map<int32_t, std::vector<MarkerObservation>> marker_storage_;
        
        // Parameters
        double confidence_threshold_;  // Green confidence threshold
        int max_observations_per_id_;  // Storage limit
        
        // Helper functions for controller functionality
        void process_marker_detection(int32_t id, double x, double y, double z, double confidence);
        void add_observation(int32_t id, double x, double y, double z, double confidence);
        void publish_averaged_positions();
        void evict_old_observations(int32_t id);

        // Declare members
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        
        // Camera calibration parameters (you may need to adjust these for your camera)
        cv::Mat camera_matrix;
        cv::Mat dist_coeffs;
        
        // Tag size in meters (adjust this based on your actual ArUco tag size)
        double tag_size;
};

#endif