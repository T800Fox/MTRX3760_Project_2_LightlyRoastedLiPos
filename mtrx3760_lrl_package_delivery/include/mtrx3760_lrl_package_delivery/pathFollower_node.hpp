#ifndef mtrx3760_lrl_package_delivery_PATHFOLLOWER_NODE_HPP_
#define mtrx3760_lrl_package_delivery_PATHFOLLOWER_NODE_HPP_

#include <memory>
#include <algorithm> 
#include <cmath>
#include <deque>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#include "mtrx3760_lrl_interfaces/action/test.hpp"
#include "mtrx3760_lrl_actuator/actuator_client.hpp"

//Msgs
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "mtrx3760_lrl_interfaces/srv/path_req.hpp"



//Alternate command types enum
enum CMD_TYPE {
    ROTATE,
    DRIVE
};

class pathFollower : public rclcpp::Node
{
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