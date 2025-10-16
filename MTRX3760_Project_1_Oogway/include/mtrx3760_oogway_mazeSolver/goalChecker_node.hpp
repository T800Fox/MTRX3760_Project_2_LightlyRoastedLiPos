#ifndef MTRX3760_OOGWAY_MAZESOLVER_GOALCHECKER_NODE_HPP_
#define MTRX3760_OOGWAY_MAZESOLVER_GOALCHECKER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "mtrx3760_oogway_mazeSolver/utils.hpp"


class goalChecker : public rclcpp::Node
{
    public:
        goalChecker();
        ~goalChecker();
    private:
        // ROS topic publishers
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr at_goal_pub_;

        // ROS topic subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        // subscription callbacks
        void odom_callback(const nav_msgs::msg::Odometry msg);
};

#endif