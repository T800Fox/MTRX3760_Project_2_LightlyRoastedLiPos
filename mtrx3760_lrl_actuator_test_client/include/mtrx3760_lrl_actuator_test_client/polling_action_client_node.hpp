#ifndef MTRX3760_LRL_WAREHOUSEBOT_POLLING_ACTION_CLIENT_NODE_HPP_
#define MTRX3760_LRL_WAREHOUSEBOT_POLLING_ACTION_CLIENT_NODE_HPP_


#include <memory>
#include <thread>
#include <vector>
 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mtrx3760_lrl_interfaces/action/test.hpp"

// #include <functional>
// #include <future>
// #include <memory>
// #include <string>
// #include <thread>
// #include <chrono>
// #include <sstream>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

// #include "mtrx3760_lrl_interfaces/action/test.hpp"

enum ACTION_STATUS {
    UNDEFINED = 0,
    AWAITING_RESPONSE = 1,
    AWAITING_RESULT = 2
};

struct actionCommand{
    int mode;
    double magnitude;
};
 
using Actuator = mtrx3760_lrl_interfaces::action::Test;
using GoalHandleActuator = rclcpp_action::ClientGoalHandle<Actuator>;
 
/**
 * @class TimedRotationActionClient
 * @brief ROS 2 action client node for sending timed rotation goals.
 *
 * This class implements a ROS 2 action client that sends timed rotation goals and
 * processes feedback and results from the action server. It can be controlled via
 * a boolean topic to start and stop rotations.
 */
class AddisonActuatorActionClient : public rclcpp::Node
{
    public:
        using Actuator = mtrx3760_lrl_interfaces::action::Test;
        using GoalHandleActuator = rclcpp_action::ClientGoalHandle<Actuator>;
        
        AddisonActuatorActionClient(const rclcpp::NodeOptions& options);

        void addCommand(actionCommand aCommand);

    
    private:
        void goal_response_callback(const GoalHandleActuator::SharedPtr& goal_handle);
        
        void feedback_callback(
            const GoalHandleActuator::SharedPtr& goal_handle,
            const std::shared_ptr<const Actuator::Feedback>& feedback);
        
        void get_result_callback(const GoalHandleActuator::WrappedResult& result);
        
        void cancel_goal();
        

        void send_goal(); // int aMode, double aMagnitude


        actionCommand fetchNextCommand();

        // Class member variables
        rclcpp_action::Client<Actuator>::SharedPtr action_client_;        ///< Action client for timed rotation
        // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr timed_rotation_mode_sub_;  ///< Subscription to timed rotation mode
        // bool timed_rotation_mode_{false};                                      ///< Current state of timed rotation mode
        GoalHandleActuator::SharedPtr goal_handle_; 

        int currentActionStatus;
        
        int storedActionMode;
        double storedActionMagnitude;

        std::vector<actionCommand> commandList;
};


// #include <functional>
// #include <future>
// #include <memory>
// #include <string>
// #include <thread>
// #include <chrono>
// #include <sstream>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

// #include "mtrx3760_lrl_interfaces/action/test.hpp"

// enum CONTROL_METHODS {
//     PASSIVE = -1,
//     ABS_LINEAR = 0,
//     ABS_ANGULAR = 1,
//     VEL_LINEAR = 2,
//     VEL_ANGULAR = 3
// };

// enum ACTION_STATUS {
//     UNDEFINED = 0,
//     AWAITING_RESPONSE = 1,
//     AWAITING_RESULT = 2
// };

// struct actionCommand{
//     int mode;
//     double magnitude;
// };

// namespace mtrx3760_lrl_warehousebot
// {
//     class ActuatorDebugClient : public rclcpp::Node
//     {
//         public:
//             using Actuator = mtrx3760_lrl_interfaces::action::Test;
//             using GoalHandleActuator = rclcpp_action::ClientGoalHandle<Actuator>;

//             ActuatorDebugClient();
//             void send_goal();

//             void sendGoalWPolling(int aControlMethod, double aMagnitude);
//             void feedActuatorActions(std::vector<actionCommand> aRoutine);
        

//         private:
//             rclcpp_action::Client<Actuator>::SharedPtr client_ptr_;
//             rclcpp::TimerBase::SharedPtr timer_;

//             int currentActionStatus;
//     };
// }  // namespace custom_action_cpp
#endif