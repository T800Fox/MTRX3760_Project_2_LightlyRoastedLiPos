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

//Planning stuff
#include "mtrx3760_lrl_package_delivery/path_finder/planning_core.h"
#include "mtrx3760_lrl_package_delivery/path_finder/commands.h"


struct Point{
    double x;
    double y;
};

struct Package{
    uint32_t ID;
    Point global_pos;
};



class DeliveryController : public rclcpp::Node {

    public:
        using PerformDelivery = mtrx3760_lrl_interfaces::action::PerformDelivery;
        using GoalHandlePerformDelivery = rclcpp_action::ServerGoalHandle<PerformDelivery>;

        // Constructor and Destructor
        DeliveryController();
        ~DeliveryController();

    private:
        // Server for perform delivery action
        rclcpp_action::Server<PerformDelivery>::SharedPtr delivery_action_server_;

        // Client to handle path request service response from Path Follower Node
        rclcpp::Client<mtrx3760_lrl_interfaces::srv::PathReq>::SharedPtr path_request_client_;
        
        // Ros subscribers
        rclcpp::Subscription<mtrx3760_lrl_interfaces::msg::MarkerDetection>::SharedPtr package_detection_sub_;
        rclcpp::Subscription<mtrx3760_lrl_interfaces::msg::Pose>::SharedPtr robot_pose_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_; // To pass the map data to the path finder

        // Callback function for marker detection
        void package_detection_callback(const mtrx3760_lrl_interfaces::msg::MarkerDetection::SharedPtr msg);
        void delivery_action_execute(const std::shared_ptr<GoalHandlePerformDelivery> goal_handle);

        //Send path request to path-follower
        void send_path_request(std::vector<MotionCmd> cmds);


        //Most recently recieved occupancy grid
        nav_msgs::msg::OccupancyGrid::SharedPtr most_recent_map;

        //Path finder member
        PlanningCore path_finder;
        mtrx3760_lrl_interfaces::msg::Pose::SharedPtr curr_pose;

        //Is accepting goals - is delivery not in progress
        bool is_accepting_goals;

        std::unordered_map<uint32_t, Package> tag_dictionary_;


};

#endif // INCLUDE_mtrx3760_lrl_package_delivery_DELIVERYCONTROLLER_NODE_HPP_