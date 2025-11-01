#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mtrx3760_oogway_mazesolver/path_finder/planning_core.h"
#include "mtrx3760_oogway_mazesolver/path_finder/commands.h"

// #include "mtrx3760_lrl_warehousebot/srv/path_req.hpp"

#ifndef INCLUDE_MTRX3760_LRL_WAREHOUSEBOT_DELIVERYCONTROLLER_NODE_HPP_
#define INCLUDE_MTRX3760_LRL_WAREHOUSEBOT_DELIVERYCONTROLLER_NODE_HPP


class DeliveryController : public rclcpp::Node {

    public:
        using PerformDelivery = lrl_action_interface::action::InspectWarehouse;
        using GoalHandlePerformDelivery = rclcpp_action::ServerGoalHandle<InspectWarehouse>;

        // Constructor and Destructor
        DeliveryController();
        ~DeliveryController();

    private:
        // Server for perform delivery action
        rclcpp_action::Server<PerformDelivery>::SharedPtr delivery_action_server_;

        // Client to handle path request service response from Path Follower Node
        rclcpp::Client<mtrx3760_lrl_warehousebot::srv::PathReq>::SharedPtr path_request_client_;
        
        // Ros subscribers
        rclcpp::Subscription<mtrx3760_lrl_warehousebot::msg::PackageDetection>::SharedPtr package_detection_subscriber_; // To receive package detection data from camera node
        rclcpp::Subscription<mtrx3760_lrl_warehousebot::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_; // To pass the map data to the path finder

        // Callback function for marker detection
        void package_detection_callback(const mtrx3760_lrl_warehousebot::msg::PackageDetection::SharedPtr msg);

        //Send path request to path-follower
        void send_path_request(const std::vector<double>& angles, const std::vector<double>& distances);


        //Most recently recieved occupancy grid
        mtrx3760_lrl_warehousebot::msg::OccupancyGrid::SharedPtr most_recent_map;

        //Path finder member
        PlanningCore path_finder;


};

#endif // INCLUDE_MTRX3760_LRL_WAREHOUSEBOT_DELIVERYCONTROLLER_NODE_HPP_