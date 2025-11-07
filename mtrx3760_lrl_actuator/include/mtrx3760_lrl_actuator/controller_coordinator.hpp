// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: controller_coordinator
// Author(s): Jeremy Fox
//
// Is the interface for passing instructions to a currently running controller,
// and getting stamped twist commands back to the actuator action server.
//
// To do this it keeps track of the controller types, and which one, if any, are running.
//
// If no absolute controller (or any other controller that takes /tf as feedback) is 
// running, it will push a stamped twist containing default velocities. As the tf callback
// in the node always needs a stamped twist. 
// Upon recieving a velocity command, these passive values are updated to match it

#ifndef MTRX3760_LRL_WAREHOUSEBOT_CONTROLLER_COORDINATOR_HPP_
#define MTRX3760_LRL_WAREHOUSEBOT_CONTROLLER_COORDINATOR_HPP_
//---Includes
//--Vanilla C++--
#include <iostream>

//--ROS Action Dependancies--
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//--ROS Messages and Interfaces--
#include <geometry_msgs/msg/transform_stamped.hpp>          // 
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "mtrx3760_lrl_interfaces/action/test.hpp"

//--Custom Turtlebot Controller Library
#include "mtrx3760_lrl_actuator/controller_variants.hpp"


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

        controllerCoordinator();        // start with passive veloctites at zero 
        ~controllerCoordinator();       // cleans up dynamically allocated controller if running at destruction

        bool controllerRunning();                                                       // means the node can know if a controller is running, allows for accepting or rejecting goals
        void fireUpController(const std::shared_ptr<GoalHandleActuator> goal_handle);   // recieves a goal handle and orchestrates the setup for a conroller

        geometry_msgs::msg::TwistStamped updatePassiveVelLinear(const std::shared_ptr<GoalHandleActuator> goal_handle); // for direct control over velocity, mazeNavigator handles control w/ scan data
        geometry_msgs::msg::TwistStamped updatePassiveVelAngular(const std::shared_ptr<GoalHandleActuator> goal_handle);// ^ ditto. ^

        // gets a controllers response for a given position (tf) out to the node for publishing, if a controller's response has a magnitude of zero, clean up
        geometry_msgs::msg::TwistStamped routeTf(geometry_msgs::msg::TransformStamped msg); 
    private:
        //--Enums--
        enum CONTROL_METHODS {
            PASSIVE = -1,
            ABS_LINEAR = 0,
            ABS_ANGULAR = 1
        };
        
        //--Variables--
        controller *currentController;  // pointer to the controller currently being used, NULL ptr when nothing being used
        int currentControlMethod;       // stores current controller using CONTROL_METHODS enum
        double passiveAngularVelocity;  // angular velocity to pass if no controller running
        double passiveLinearVelocity;   // linear velocity to pass if no controller running
    };
    
};

#endif