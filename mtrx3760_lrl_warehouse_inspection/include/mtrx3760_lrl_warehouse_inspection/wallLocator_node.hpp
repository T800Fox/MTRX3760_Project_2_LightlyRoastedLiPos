#ifndef MTRX3760_OOGWAY_MAZESOLVER_WALLFINDER_NODE_HPP_
#define MTRX3760_OOGWAY_MAZESOLVER_WALLFINDER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "mtrx3760_oogway_mazesolver/msg/wall_dist.hpp"

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

//Threshold distance at which robot is considered to be in contact with wall
const float DIST_THRESHOLD_F = 0.25; //Forwards
const float DIST_THRESHOLD_S = 0.35; //Sidesways


class wallLocator : public rclcpp::Node
{
    public:
        wallLocator();
        ~wallLocator();
    private:
        // ROS topic publishers
        rclcpp::Publisher<mtrx3760_oogway_mazesolver::msg::WallDist>::SharedPtr wall_dist_pub_;

        // ROS topic subscribers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

        // subscription callbacks
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        // helper functions
        void wall_dist_publisher(const sensor_msgs::msg::LaserScan::SharedPtr data);

        
};

#endif