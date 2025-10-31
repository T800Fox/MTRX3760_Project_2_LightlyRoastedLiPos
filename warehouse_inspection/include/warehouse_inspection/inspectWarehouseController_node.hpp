
#ifndef MTRX3760_OOGWAY_INSPECT_WAREHOUSE_CONTROLLER_NODE_HPP_
#define MTRX3760_OOGWAY_INSPECT_WAREHOUSE_CONTROLLER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "wallFollower.hpp"

#include "lrl_action_interface/action/inspect_warehouse.hpp"


class inspectWarehouseController : public rclcpp::Node
{
    public:
        using InspectWarehouse = lrl_action_interface::action::InspectWarehouse;
        using GoalHandleInspectWarehouse = rclcpp_action::ServerGoalHandle<InspectWarehouse>;
        using Actuator = lrl_action_interface::action::Test;
        using GoalHandleActuator = rclcpp_action::ClientGoalHandle<Actuator>;

        inspectWarehouseController();
        ~inspectWarehouseController();

    private:
        rclcpp_action::Server<InspectWarehouse>::SharedPtr inspection_action_server_;

        rclcpp_action::Client<Actuator>::SharedPtr actuator_client_;

        //Logic class for calculating actuator command
        wallFollower wall_follower_logic;



};

#endif