// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: controller_coordinator
// Author(s): Jeremy Fox
//
// Acts the interface for passing enviroment and state variables, from the actutor node, to a running controller.
// Also keeps track of the controller types and if they're running.

#ifndef MTRX3760_LRL_WAREHOUSEBOT_CONTROLLER_COORDINATOR_HPP_
#define MTRX3760_LRL_WAREHOUSEBOT_CONTROLLER_COORDINATOR_HPP_
//---Includes
//--Vanilla C++--
#include <iostream>

//--ROS Action Dependancies--
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//--ROS Messages and Interfaces--
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "mtrx3760_lrl_interfaces/action/test.hpp"

//--Custom Turtlebot Controller Library
#include "mtrx3760_lrl_actuator/controller_variants.hpp"


//---Enums---
enum CONTROL_METHODS {
    PASSIVE = -1,
    ABS_LINEAR = 0,
    ABS_ANGULAR = 1,
    VEL_LINEAR = 2,
    VEL_ANGULAR = 3
};


//---Classes---
namespace mtrx3760_lrl_warehousebot
{
    // acts as middleman between action node and a controller. 
    // set's up a controller, keeps track of it, routes inputs/outputs, then cleans ready for the next one.
    class controllerCoordinator
    {
    public:
        using Actuator = mtrx3760_lrl_interfaces::action::Test;
        using GoalHandleActuator = rclcpp_action::ServerGoalHandle<Actuator>;    

        controllerCoordinator();
        ~controllerCoordinator();

        bool controllerRunning();                                                       // means the node knows if a controller is running, allows for accepting or rejecting goals
        void fireUpController(const std::shared_ptr<GoalHandleActuator> goal_handle);   // recieves a goal handle and orchestrates the setup for a conroller

        geometry_msgs::msg::TwistStamped updatePassiveVelLinear(const std::shared_ptr<GoalHandleActuator> goal_handle); // for direct control over velocity, mazeNavigator handles control w/ scan data
        geometry_msgs::msg::TwistStamped updatePassiveVelAngular(const std::shared_ptr<GoalHandleActuator> goal_handle);// ^ ditto. ^

        geometry_msgs::msg::TwistStamped routeTf(geometry_msgs::msg::TransformStamped msg); // gets a controllers response for a given position (tf) out to the node for publishing
    private:
        //--Variables--
        controller *currentController;  // pointer to the controller currently being used, NULL ptr when nothing being used

        int currentControlMethod;       // stores current controller using CONTROL_METHODS enum

        double passiveAngularVelocity;  // angular velocity to pass if no controller running
        double passiveLinearVelocity;   // linear velocity to pass if no controller running
    };
    
};

#endif