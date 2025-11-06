// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: deliveryController_node.hpp
// Author(s): Mudakir Sultan
//
// Header file for the deliveryController node class. Defines the deliveryController
// class for managing package delivery tasks. Uses ROS 2 for communication and
// coordination with other nodes.

#ifndef INCLUDE_mtrx3760_lrl_package_delivery_DELIVERYCONTROLLER_NODE_HPP_
#define INCLUDE_mtrx3760_lrl_package_delivery_DELIVERYCONTROLLER_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <unordered_map>

//Msg
#include "mtrx3760_lrl_interfaces/msg/marker_detection.hpp"
#include "mtrx3760_lrl_interfaces/msg/pose.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>


//Action
#include "mtrx3760_lrl_interfaces/action/perform_delivery.hpp"
#include "mtrx3760_lrl_interfaces/srv/path_req.hpp"

//Path Planning 
#include "mtrx3760_lrl_package_delivery/path_finder/planning_core.h"
#include "mtrx3760_lrl_package_delivery/path_finder/commands.h"


// ============================================================================
// Delivery Controller Class
// ============================================================================
// Class for Delivery controller operations. Provides common functionality
// for managing package delivery tasks.
class DeliveryController : public rclcpp::Node {

    public:
        // Type aliases for action server
        using PerformDelivery = mtrx3760_lrl_interfaces::action::PerformDelivery;
        using GoalHandlePerformDelivery = rclcpp_action::ServerGoalHandle<PerformDelivery>;

        // Constructor and Destructor
        DeliveryController();
        ~DeliveryController();

    private:
        // Server for perform delivery action
        rclcpp_action::Server<PerformDelivery>::SharedPtr delivery_action_server_;

        // Control flag for accepting new goals
        bool is_accepting_goals;

        // Client to handle path request service response from Path Follower Node
        rclcpp::Client<mtrx3760_lrl_interfaces::srv::PathReq>::SharedPtr path_request_client_;

        // ROS subscribers
        rclcpp::Subscription<mtrx3760_lrl_interfaces::msg::MarkerDetection>::SharedPtr package_detection_sub_;
        rclcpp::Subscription<mtrx3760_lrl_interfaces::msg::Pose>::SharedPtr robot_pose_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_; 

        // Most recently received occupancy grid
        nav_msgs::msg::OccupancyGrid::SharedPtr most_recent_map;

        // Robot current pose
        mtrx3760_lrl_interfaces::msg::Pose::SharedPtr curr_pose;

        // Dictionary to map packages and their priorities
        std::unordered_map<uint32_t, Package> tag_dictionary_;

          // Path finder member
        PlanningCore path_finder;

        // Callback function for package detection
        void package_detection_callback(const mtrx3760_lrl_interfaces::msg::MarkerDetection::SharedPtr msg); 

        // Method to perform delivery action
        void delivery_action_execute(const std::shared_ptr<GoalHandlePerformDelivery> goal_handle);             

        // Send path request to path-follower
        void send_path_request(std::vector<MotionCmd> cmds);

        // Struct to store robot position
        struct Point{
            double x;
            double y;
        };

        // Struct to store package information
        struct Package{
            uint32_t ID;
            Point global_pos;
        };
};

#endif // INCLUDE_mtrx3760_lrl_package_delivery_DELIVERYCONTROLLER_NODE_HPP_