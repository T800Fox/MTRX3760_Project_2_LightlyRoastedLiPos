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
#include <marker_tracking/msg/marker_position.hpp>

using MarkerPosition = marker_tracking::msg::MarkerPosition;

struct MarkerObservation {
    double x, y, z;
    double confidence;
    rclcpp::Time timestamp;
    
    MarkerObservation(double x_, double y_, double z_, double conf_, rclcpp::Time t)
        : x(x_), y(y_), z(z_), confidence(conf_), timestamp(t) {}
};

// Running statistics per marker ID (incremental mean and variance)
struct RunningMarkerStats {
    int32_t count = 0;
    double mean_x = 0.0;
    double mean_y = 0.0;
    double mean_z = 0.0;
    double m2_x = 0.0; // sum of squared deltas for variance (Welford)
    double m2_y = 0.0;
    double m2_z = 0.0;
    double mean_confidence = 0.0;
};

// Base Camera class for general camera operations
class Camera : public rclcpp::Node
{
    public:
        Camera(const std::string& node_name);
        virtual ~Camera();
        
    protected:
        // ROS topic publisher
        rclcpp::Publisher<MarkerPosition>::SharedPtr marker_pub_;
        
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
        
        // Storage: map of ID -> running stats (no full history)
        std::map<int32_t, RunningMarkerStats> marker_stats_;
        
        // Storage: map of ID -> closest image and distance
        std::map<int32_t, cv::Mat> closest_images_;
        std::map<int32_t, double> closest_distances_;
        
        // Parameters
        double confidence_threshold_;  // Green confidence threshold
        int max_observations_per_id_;  // Deprecated: no longer used
        
        // Helper functions for controller functionality
        void process_marker_detection(int32_t id, double x, double y, double z, double confidence);
        void add_observation(int32_t id, double x, double y, double z, double confidence);

        // Declare members
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        
        // Camera calibration parameters (you may need to adjust these for your camera)
        cv::Mat camera_matrix;
        cv::Mat dist_coeffs;
        
        // Tag size in meters (adjust this based on your actual ArUco tag size)
        double tag_size;
};

#endif