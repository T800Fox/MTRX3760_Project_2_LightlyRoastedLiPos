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

//---Constants---
const double pi = 3.14159265359;
const double MAX_ANG_VEL = 0.8;
const double MAX_LIN_VEL = 0.2;

//---Classes---
namespace mtrx3760_lrl_warehousebot
{
    // Abstract base class defines the interface for the controllers.
    class controller
    {
        public:
            // namespace abbreviations
            using Actuator = mtrx3760_lrl_interfaces::action::Test;
            using GoalHandleActuator = rclcpp_action::ServerGoalHandle<Actuator>;

            // intialises operation variable to safe values, no destructor as no dynamic memory to look after
            controller(const std::shared_ptr<GoalHandleActuator> goal_handle);
            
            // the interface where any controller's next stamped twist response can be produced
            virtual geometry_msgs::msg::TwistStamped respondToStimulus(geometry_msgs::msg::TransformStamped aTransformMsg) = 0;

        protected:
            //--Helper Methods--
            // was used by angular and, angular+linear controller. only used by angular but may be used by other future controllers
            double transformToHeading(geometry_msgs::msg::TransformStamped aTransformMsg);

            //--Enums--
            // denotes the states of operation for a controller
            enum CONTROL_STATE {
                AWAITING_START_COND = 0,
                RUNNING = 1,
                TARGET_REACHED = 2,
                COMPLETE = 3,
                ABORT = -1
            };

            //--Variables--
            std::shared_ptr<GoalHandleActuator> controllerGoalHandle;   // makes goal handle accessible to inheritors
            int storedControlState;                                     // all controllers have a controll state
    };
    
    // class to encapsulate a controller that offsets the turtlebot's heading by a given number of radians
    class absAngularController : public controller
    {
        public:
            
            absAngularController(const std::shared_ptr<GoalHandleActuator> goal_handle);

            // run through one loop of control with a given transform
            geometry_msgs::msg::TwistStamped respondToStimulus(geometry_msgs::msg::TransformStamped aTransformMsg) override;

        private:
            //--Helper Methods--
            // assuming this transform is the first one, pull out the heading and use it to compute the target
            double generateTargetValue(geometry_msgs::msg::TransformStamped aTransformMsg); 
            // get the size of the error, doJump will apply a correction if the heading moves over the -pi / pi boundary           
            double generateErrorTerm(geometry_msgs::msg::TransformStamped aTransformMsg, bool doJump); 
            // calculate the proportional response to the error given, ensures no values above max
            double computeResponse(double aErrorVal); 

            //--Variables--
            //-state-
            double targetValue;             // heading angle to reach (rad.)
            // used when path takes robot over -pi / pi boundary, if it's around this point and the error has a massive spike the controller knows not to freak out
            double lastError = pi / 2.0;    
            //-tuning-
            // may be feasible to controller to update it's own trim values? decided to place them in the controller with default values
            double p = 0.75;                // response to error magnitude
            double completionTol = 0.1;     // how close to get before considering the actuation complete (rad.)
    };

    // class that encapsulates a controller that translates the turtlebot forwards a given distance
    class absLinearController : public controller
    {
        public:
            // extracts and stores distance to drive from goal handle, no need for destructor as no dynamic memory
            absLinearController(const std::shared_ptr<GoalHandleActuator> goal_handle);         
            // run through one loop of control with a given transform
            geometry_msgs::msg::TwistStamped respondToStimulus(geometry_msgs::msg::TransformStamped aTransformMsg) override;

        private:
            //--Enums--
            // abstraction of cartesian coordinates
            struct coord
            {
                double x;
                double y;
            };

             //--Helper Methods--    
            coord transformToCoord(geometry_msgs::msg::TransformStamped aTransformMsg);         // extracts the coord from a given transform
            double generateErrorTerm(geometry_msgs::msg::TransformStamped aTransformMsg);       // compares current coord's to starting ar resolves error
            double computeResponse(double aErrorVal);                                           // from a given error computes a pd response

            //--Variables--
            //-state-
            coord startCoord;       // taken from stamped transform at first tf callback
            double goalDistance;    // taken from goal handle
            double lastErr;         // stored during control; approximation of error's derivatrive
            //-tuning-
            // may be feasible to controller to update it's own trim values? decided to place them in the controller with default values
            double p = 0.5;         // response to error size
            double d = 0.2;         // response to error gradient
            double completionTol = 0.01;   // smallest velocity to be sent
    };
};

#endif
