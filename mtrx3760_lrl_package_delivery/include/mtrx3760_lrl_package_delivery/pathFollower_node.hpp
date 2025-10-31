#ifndef MTRX3760_OOGWAY_MAZESOLVER_PATHFOLLOWER_NODE_HPP_
#define MTRX3760_OOGWAY_MAZESOLVER_PATHFOLLOWER_NODE_HPP_

#include <memory>
#include <algorithm> 
#include <cmath>
#include <deque>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

//Msgs
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "mtrx3760_lrl_interfaces/srv/path_req.hpp"



//Alternate command types enum
enum CMD_TYPE {
    ROTATE,
    DRIVE
};

class pathFollower : public rclcpp::Node
{
    public:
        pathFollower();
        ~pathFollower();
    private:
        // ROS topic publishers
        rclcpp::Publisher<mtrx3760_oogway_mazesolver::msg::AngularCmd>::SharedPtr angular_cmd_pub_;
        rclcpp::Publisher<mtrx3760_oogway_mazesolver::msg::LinearCmd>::SharedPtr linear_cmd_pub_;


        // ROS topic subscribers
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_abs_rotating_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_abs_moving_sub_;
        

        // Service servers
        rclcpp::Service<mtrx3760_oogway_mazesolver::srv::PathReq>::SharedPtr follow_path_server;

        // Service callbacks
        void follow_path_callback(
            const std::shared_ptr<mtrx3760_oogway_mazesolver::srv::PathReq::Request> request,
            std::shared_ptr<mtrx3760_oogway_mazesolver::srv::PathReq::Response> response);


        // Member variables
        bool is_abs_rotating;
        bool is_abs_moving;

        std::vector<double> distances;
        std::vector<double> angles;
        unsigned int cmd_ind; //Index of current command

        double curr_angle;

        // Member functions
        void update();
        
};




#endif