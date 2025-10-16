#ifndef MTRX3760_OOGWAY_MAZESOLVER_CAMERA_NODE_HPP_
#define MTRX3760_OOGWAY_MAZESOLVER_CAMERA_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "mtrx3760_oogway_mazeSolver/utils.hpp"

class camera : public rclcpp::Node
{
    public:
        camera();
        ~camera();
    private:
        
        // ROS topic subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;

        // subscription callbacks
        void camera_callback(const sensor_msgs::msg::Image::SharedPtr image);
};

#endif