
#ifndef MTRX3760_LRL_INSPECT_WAREHOUSE_CONTROLLER_NODE_HPP_
#define MTRX3760_LRL_INSPECT_WAREHOUSE_CONTROLLER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "mtrx3760_lrl_warehouse_inspection/msg/wall_dist.hpp"
#include "mtrx3760_lrl_warehouse_inspection/msg/pose.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "mtrx3760_lrl_interfaces/action/inspect_warehouse.hpp"
#include "mtrx3760_lrl_interfaces/action/test.hpp"

#include "mtrx3760_lrl_actuator/actuator_client.hpp"

#include "mtrx3760_lrl_warehouse_inspection/wallFollower.hpp"
#include "mtrx3760_lrl_warehouse_inspection/msg/wall_dist.hpp"


class inspectWarehouseController : public rclcpp::Node
{
    public:
        using InspectWarehouse = mtrx3760_lrl_interfaces::action::InspectWarehouse;
        using GoalHandleInspectWarehouse = rclcpp_action::ServerGoalHandle<InspectWarehouse>;
        using Actuator = mtrx3760_lrl_interfaces::action::Test;
        using GoalHandleActuator = rclcpp_action::ClientGoalHandle<Actuator>;

        inspectWarehouseController();
        ~inspectWarehouseController();

    private:
        //Ros subs
        rclcpp::Subscription<mtrx3760_lrl_warehouse_inspection::msg::WallDist>::SharedPtr wall_dist_sub_;
        rclcpp::Subscription<mtrx3760_lrl_warehouse_inspection::msg::Pose>::SharedPtr curr_pose_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        //Pubs
        rclcpp::Publisher<mtrx3760_lrl_warehouse_inspection::msg::Pose>::SharedPtr refined_pose_pub_;

        //Action client and server
        rclcpp_action::Server<InspectWarehouse>::SharedPtr inspection_action_server_;
        rclcpp_action::Client<Actuator>::SharedPtr actuator_client_;

        //Callbacks
        void wall_dist_callback(const mtrx3760_lrl_warehouse_inspection::msg::WallDist::SharedPtr msg);

        //Members
        //Logic class for calculating actuator command
        wallFollower wall_follower_logic;

        ActuatorClientWrapper actuator_client_wrapper_;

        Pose2D refined_pose;
        Pose2D curr_pose;

        rclcpp_action::Client<Actuator>::SharedPtr test_client_ptr_;
        rclcpp::TimerBase::SharedPtr delayed_wrapper_init_timer_;

        
        double offset_x, offset_y, rot_offset;




};

#endif