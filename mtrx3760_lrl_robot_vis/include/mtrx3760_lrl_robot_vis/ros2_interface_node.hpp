#ifndef ROBOT_VIS_ROS2_INTERFACE_NODE_HPP_
#define ROBOT_VIS_ROS2_INTERFACE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "mtrx3760_lrl_interfaces/msg/marker_detection.hpp"
#include "mtrx3760_lrl_interfaces/msg/line_seg.hpp"
#include "mtrx3760_lrl_warehouse_inspection/msg/pose.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "base64.h"
#include "mtrx3760_lrl_robot_vis/render_objs.hpp"
#include "mtrx3760_lrl_robot_vis/visualiser.hpp"
#include "mtrx3760_lrl_robot_vis/socket.hpp"
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

using MarkerDetection = mtrx3760_lrl_interfaces::msg::MarkerDetection;

class Ros2Interface : public rclcpp::Node
{
    public:
        Ros2Interface();
        ~Ros2Interface();

    private:
        // ROS topic subscribers
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<MarkerDetection>::SharedPtr package_detection_sub_;
        rclcpp::Subscription<mtrx3760_lrl_interfaces::msg::LineSeg>::SharedPtr wall_follower_seg_sub_;
        rclcpp::Subscription<mtrx3760_lrl_warehouse_inspection::msg::Pose>::SharedPtr refined_pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

        // Ros timer
        rclcpp::TimerBase::SharedPtr update_timer;

        // subscription callbacks
        void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

        void refined_pose_callback(const mtrx3760_lrl_warehouse_inspection::msg::Pose::SharedPtr msg);
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void package_detection_callback(const MarkerDetection::SharedPtr msg);
        void wall_follower_seg_callback(const mtrx3760_lrl_interfaces::msg::LineSeg::SharedPtr msg);
        
        void update_callback();

        //TCP connection
        TCP_Connection interface_socket;



};







#endif