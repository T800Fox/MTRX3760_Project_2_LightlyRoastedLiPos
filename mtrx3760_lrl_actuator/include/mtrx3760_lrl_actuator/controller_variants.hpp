// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: controller_variants
// Author(s): Jeremy Fox
//
// Defines the controllers that can be used by the controller coordinator, building them from an abstact base class 'controller'.
// All of the controllers work off the base format; 
//      1. Recieve and parse new position data, Compute and store goal state if you haven't already
//      2. Compute current error 
//      3. Use control method and error to decide a response
//      4. Put response in a Twist Stamp, so that the actuator action server node can publish it to cmd_vel.
//      
// This design was chosen to make prototyping new controllers as easy as possible.

#ifndef MTRX3760_LRL_WAREHOUSEBOT_CONTROLLER_VARIANTS_HPP_
#define MTRX3760_LRL_WAREHOUSEBOT_CONTROLLER_VARIANTS_HPP_
//---Includes
//--Vanilla C++--
#include <vector>
#include <iostream>
#include <cmath>
//--ROS Action Dependancies--
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//--Quaternion ROS Dependacies
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
//--ROS Messages and Interfaces--
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "mtrx3760_lrl_interfaces/action/test.hpp"

//---Enums---
enum CONTROL_STATE {
    AWAITING_START_COND = 0,
    RUNNING = 1,
    TARGET_REACHED = 2,
    COMPLETE = 3,
    ABORT = -1
};

struct coord
{
    double x;
    double y;
};


//---Constants---
const double pi = 3.14159265359;
const double MAX_ANG_VEL = 1.1;
const double MAX_LIN_VEL = 0.2;

//---Classes---
namespace mtrx3760_lrl_warehousebot
{
    // Abstract base class defines the interface for the controllers.
    class controller
    {
        public:
            using Actuator = mtrx3760_lrl_interfaces::action::Test;
            using GoalHandleActuator = rclcpp_action::ServerGoalHandle<Actuator>;

            controller(const std::shared_ptr<GoalHandleActuator> goal_handle);
            ~controller();
            
            virtual geometry_msgs::msg::TwistStamped respondToStimulus(geometry_msgs::msg::TransformStamped aTransformMsg) = 0;

        protected:
            //--Helper Methods--
            double transformToHeading(geometry_msgs::msg::TransformStamped aTransformMsg);


            //--Variables--
            std::shared_ptr<GoalHandleActuator> controllerGoalHandle;
            int storedControlState;

            double targetValue;
    };
    
    class absAngularController : public controller
    {
        public:
            // using Actuator = lrl_action_interface::action::Test;
            // using GoalHandleActuator = rclcpp_action::ServerGoalHandle<Actuator>;   
            
            absAngularController(const std::shared_ptr<GoalHandleActuator> goal_handle);
            ~absAngularController();

            geometry_msgs::msg::TwistStamped respondToStimulus(geometry_msgs::msg::TransformStamped aTransformMsg) override;

        private:
            //--Helper Methods--
            double generateTargetValue(geometry_msgs::msg::TransformStamped aTransformMsg);
            double generateErrorTerm(geometry_msgs::msg::TransformStamped aTransformMsg, bool doJump);
            double computeResponse(double aErrorVal);


            //--Variables--
            double p = 0.75;
            double completionTol = 0.1; // TODO:  move to better spot!!!!
            double lastError = pi / 2.0;
    };

    class absLinearController : public controller
    {
        public:
            // using Actuator = lrl_action_interface::action::Test;
            // using GoalHandleActuator = rclcpp_action::ServerGoalHandle<Actuator>;

            absLinearController(const std::shared_ptr<GoalHandleActuator> goal_handle);
            ~absLinearController();

            geometry_msgs::msg::TwistStamped respondToStimulus(geometry_msgs::msg::TransformStamped aTransformMsg) override;

        private:
            //--Helper Methods--
            coord generateTargetCoord(geometry_msgs::msg::TransformStamped aTransformMsg);
            coord transformToCoord(geometry_msgs::msg::TransformStamped aTransformMsg);
            double generateErrorTerm(geometry_msgs::msg::TransformStamped aTransformMsg);
            double computeResponse(double aErrorVal);
            double computeHeadingToTarget(geometry_msgs::msg::TransformStamped aTransformMsg);


            //--Variables--

            // coord targetCoord;
            coord startCoord;

            double goalDistance;

            double p = 0.5;
            double d = 0.2;
            double completionTol = 0.01;
            double lastErr;
    };
};

#endif
