#ifndef ROBOT_VIS_ROS2_INTERFACE_NODE_HPP_
#define ROBOT_VIS_ROS2_INTERFACE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "render_objs.hpp"
#include <cmath>


struct ros_data{
  Point robot_pos;
  double robot_rot;
};

extern ros_data shared_data;

class Ros2Interface : public rclcpp::Node
{
    public:
        Ros2Interface();
        ~Ros2Interface();

    private:
        // ROS topic subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        // subscription callbacks
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};







#endif