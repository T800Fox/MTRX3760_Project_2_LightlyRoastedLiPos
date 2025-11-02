#ifndef MARKER_TRACKING_CONTROLLER_NODE_HPP_
#define MARKER_TRACKING_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mtrx3760_lrl_camera/msg/detected_marker.hpp>
#include <mtrx3760_lrl_camera/msg/marker_positions.hpp>
#include <map>
#include <vector>
#include <memory>

using DetectedMarker = mtrx3760_lrl_camera::msg::DetectedMarker;
using MarkerPositions = mtrx3760_lrl_camera::msg::MarkerPositions;
using MarkerPosition = mtrx3760_lrl_camera::msg::MarkerPosition;

struct MarkerObservation {
    double x, y, z;
    double confidence;
    rclcpp::Time timestamp;
    
    MarkerObservation(double x_, double y_, double z_, double conf_, rclcpp::Time t)
        : x(x_), y(y_), z(z_), confidence(conf_), timestamp(t) {}
};

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode();
    ~ControllerNode();

private:
    // Subscribers
    rclcpp::Subscription<DetectedMarker>::SharedPtr detection_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Publisher
    rclcpp::Publisher<MarkerPositions>::SharedPtr positions_pub_;
    
    // Callbacks
    void detection_callback(const DetectedMarker::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Current robot pose
    double robot_x, robot_y, robot_theta;
    
    // Storage: map of ID -> vector of observations
    std::map<int32_t, std::vector<MarkerObservation>> marker_storage_;
    
    // Parameters
    double confidence_threshold_;  // Green confidence threshold
    int max_observations_per_id_;  // Storage limit
    
    // Helper functions
    void add_observation(int32_t id, double x, double y, double z, double confidence);
    void publish_averaged_positions();
    void evict_old_observations(int32_t id);
};

#endif

