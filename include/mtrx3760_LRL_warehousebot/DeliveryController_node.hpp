#include <memory>
#include "rclcpp/rclcpp.hpp"
// #include "mtrx3760_lrl_warehousebot/srv/path_req.hpp"

#ifndef INCLUDE_MTRX3760_LRL_WAREHOUSEBOT_DELIVERYCONTROLLER_NODE_HPP_
#define INCLUDE_MTRX3760_LRL_WAREHOUSEBOT_DELIVERYCONTROLLER_NODE_HPP


class DeliveryController : public rclcpp::Node {

    public:
        // Constructor and Destructor
        DeliveryController();
        ~DeliveryController();

    private:
        // Callback function for marker detection
        void markerCallback(const mtrx3760_lrl_warehousebot::msg::PackageDetection::SharedPtr msg);

        // Callback function for user interface commands (Package list, priorities)
        void interfaceCallback(const mtrx3760_lrl_warehousebot::msg::UICommand::SharedPtr msg);

        // // CHANGE THIS TO A NORMAL METHOD BECAUSE U ONLY WANNA PASS THE WHOLE FINISHED MAP AT THE END AND ONCE 
        void occupancyGridCallback(const mtrx3760_lrl_warehousebot::msg::OccupancyGrid::SharedPtr msg);

        // Callback function for path request service response from Path Follower Node
        void pathRequestResponseCallback(rclcpp::Client<mtrx3760_lrl_warehousebot::srv::PathReq>::SharedPtr msg);

        // Client to handle path request service response from Path Follower Node
        rclcpp::Client<mtrx3760_lrl_warehousebot::srv::PathReq>::SharedPtr path_request_client_;

        // ros subs
        rclcpp::Subscription<mtrx3760_lrl_warehousebot::msg::PackageDetection>::SharedPtr package_subscriber_; // To receive package detection data from camera node
        rclcpp::Subscription<mtrx3760_lrl_warehousebot::msg::UICommand>::SharedPtr interface_subscriber_; // To receive user interface commands (might change to service later)
        rclcpp::Subscription<mtrx3760_lrl_warehousebot::msg::OccupancyGrid>::SharedPtr occupancygrid_subscriber_; // To pass the map data to the path finder

        // Request from Delivery Controller node client to Path Follower Node service
        void sendPathRequest(const std::vector<double>& angles, const std::vector<double>& distances);
        void pathRequestResponseCallback(rclcpp::Client<mtrx3760_lrl_warehousebot::srv::PathReq>::SharedPtr msg);



};

#endif // INCLUDE_MTRX3760_LRL_WAREHOUSEBOT_DELIVERYCONTROLLER_NODE_HPP_