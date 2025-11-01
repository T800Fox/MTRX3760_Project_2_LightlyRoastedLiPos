#ifndef ROBOT_VIS_ROS2_INTERFACE_NODE_HPP_
#define ROBOT_VIS_ROS2_INTERFACE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <custom_interfaces/msg/package_detection.hpp>
#include <custom_interfaces/msg/line_seg.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "mtrx3760_lrl_interfaces/msg/marker_detection.hpp"


#include "base64.h"
#include "render_objs.hpp"
#include "visualiser.hpp"
#include "socket.hpp"
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

using MarkerPosition = mtrx3760_lrl_camera::msg::MarkerPosition;

class Ros2Interface : public rclcpp::Node
{
    public:
        Ros2Interface();
        ~Ros2Interface();

    private:
        // ROS topic subscribers
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<custom_interfaces::msg::PackageDetection>::SharedPtr package_detection_sub_;
        rclcpp::Subscription<custom_interfaces::msg::LineSeg>::SharedPtr wall_follower_seg_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

        // Ros timer
        rclcpp::TimerBase::SharedPtr update_timer;

        // subscription callbacks
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void package_detection_callback(const custom_interfaces::msg::PackageDetection::SharedPtr msg);
        void wall_follower_seg_callback(const custom_interfaces::msg::LineSeg::SharedPtr msg);
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void update_callback();

        //TCP connection
        TCP_Connection interface_socket;


};







#endif