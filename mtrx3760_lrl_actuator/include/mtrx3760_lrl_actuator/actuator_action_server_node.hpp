// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: actuator_action_server
// Author(s): Jeremy Fox
//
// Setting up a ROS2 Action Server that listens to the /tf and can publish to /cmd_vel.
// Feeds tf data into a controllerCoordinator, and gets out TwistStamped messages to publish to /cmd_vel
#ifndef MTRX3760_LRL_WAREHOUSEBOT_ACTUATOR_ACTION_SERVER_NODE_HPP_
#define MTRX3760_LRL_WAREHOUSEBOT_ACTUATOR_ACTION_SERVER_NODE_HPP_

//---Includes---
//--Vanilla C++--
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

//--ROS Action Node Dependancies--
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

//--ROS TF Dependancies--
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

//--ROS Messages and Interfaces--
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "mtrx3760_lrl_interfaces/action/test.hpp"

#include "mtrx3760_lrl_actuator/controller_coordinator.hpp"


//---Classes---
using namespace std::chrono_literals;

namespace mtrx3760_lrl_warehousebot
{
    class ActuatorActionServer: public rclcpp::Node
    {
        public:
            using Actuator = mtrx3760_lrl_interfaces::action::Test;
            using GoalHandleActuator = rclcpp_action::ServerGoalHandle<Actuator>;

            ActuatorActionServer();                                                         // no destructor as no dynamic memory in class

        private:
            //--Private Methods--
            void execute(const std::shared_ptr<GoalHandleActuator> goal_handle);            // called when proceeding with a goal
            // called by the timer; if tf transform can be done, feeds into controller and publishes velocity response
            void listen_tf();   

            //--ROS Variables--
            rclcpp_action::Server<Actuator>::SharedPtr action_server_;                      // shared pointer to actuator action server
            rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_ ;   // velocity commands to turtlebot 
            std::shared_ptr<tf2_ros::Buffer> buffer;                                        // to be pulled from when TF required
            std::shared_ptr<tf2_ros::TransformListener> listener;                           // recieves TF
            rclcpp::TimerBase::SharedPtr timer;

            //--Variables--
            controllerCoordinator manager = controllerCoordinator();                        // interfacing bewteen ros layer and current controller
    }; 
} ;


#endif