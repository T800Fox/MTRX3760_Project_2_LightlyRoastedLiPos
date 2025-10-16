#ifndef MTRX3760_LRL_WAREHOUSEBOT_SENSING_NODE_HPP_
#define MTRX3760_LRL_WAREHOUSEBOT_SENSING_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>


class sensing : public rclcpp::Node
{
    public:
        sensing();
        ~sensing();
    private:
        // ROS topic publishers
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;

        // ROS topic subscribers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

        // subscription callbacks
        void imu_callback(const sensor_msgs::msg::Imu msg);
};

#endif