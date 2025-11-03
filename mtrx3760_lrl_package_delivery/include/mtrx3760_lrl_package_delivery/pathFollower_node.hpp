#ifndef mtrx3760_lrl_package_delivery_PATHFOLLOWER_NODE_HPP_
#define mtrx3760_lrl_package_delivery_PATHFOLLOWER_NODE_HPP_

#include <memory>
#include <algorithm> 
#include <cmath>
#include <deque>
#include <fstream>

//--ROS Dependancies--
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
//--ROS Interfaces--
//-Messages (Custom + Predefined)
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
//-Services-
#include "mtrx3760_lrl_interfaces/srv/path_req.hpp"
//-Actions-
#include "mtrx3760_lrl_interfaces/action/test.hpp"
#include "mtrx3760_lrl_interfaces/action/perform_delivery.hpp"
//-Custom Libraries-
#include "mtrx3760_lrl_actuator/actuator_client.hpp"
//Alternate command types enum
enum CMD_TYPE {
    ROTATE,
    DRIVE
};

class pathFollower : public rclcpp::Node
{
    using PerformDelivery = mtrx3760_lrl_interfaces::action::PerformDelivery;
    using GoalHandlePerformDelivery = rclcpp_action::ClientGoalHandle<PerformDelivery>;

    public:
        pathFollower();
        ~pathFollower();
    private:

        // Service servers
        rclcpp::Service<mtrx3760_lrl_interfaces::srv::PathReq>::SharedPtr follow_path_server;

        // Service callbacks
        void follow_path_callback(
            const std::shared_ptr<mtrx3760_lrl_interfaces::srv::PathReq::Request> request,
            std::shared_ptr<mtrx3760_lrl_interfaces::srv::PathReq::Response> response);


        //Actuator client
        ActuatorClientWrapper actuator_client_wrapper_;
        //Timer for update polling
        rclcpp::TimerBase::SharedPtr update_timer_;
        //Timer for delayed init (race con)
        rclcpp::TimerBase::SharedPtr delayed_wrapper_init_timer_;


        //Member variables
        std::vector<double> distances;
        std::vector<double> angles;
        unsigned int cmd_ind; //Index of current command



        // Member functions
        void update();
        
};




#endif