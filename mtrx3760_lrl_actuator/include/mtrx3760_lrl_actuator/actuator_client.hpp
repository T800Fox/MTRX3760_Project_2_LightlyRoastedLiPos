#ifndef MTRX3760_LRL_WAREHOUSEBOT_ACTION_CLIENT_HPP_
#define MTRX3760_LRL_WAREHOUSEBOT_ACTION_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mtrx3760_lrl_interfaces/action/test.hpp"


//Actuator goal type
struct ActuatorCmd {
    static constexpr uint8_t MODE_ABS_LINEAR  = 0;
    static constexpr uint8_t MODE_ABS_ANGULAR = 1;
    static constexpr uint8_t MODE_VEL_LINEAR  = 2;
    static constexpr uint8_t MODE_VEL_ANGULAR = 3;

    uint8_t mode;
    double magnitude;
};


//Generic wrapper for action that impliments blocking functionaility through polling (work around)
class ActuatorClientWrapper {

    enum ACTION_STATUS {
        UNDEFINED = 0,
        AWAITING_RESPONSE = 1,
        AWAITING_RESULT = 2
    };

    public:
        using Actuator = mtrx3760_lrl_interfaces::action::Test;
        using GoalHandleActuator = rclcpp_action::ClientGoalHandle<Actuator>;


        ActuatorClientWrapper() = default;
        ActuatorClientWrapper(rclcpp::Node::SharedPtr node, const std::string &action_name);
        


        bool wait_for_server(std::chrono::seconds timeout = std::chrono::seconds(3));

        void send_goal(ActuatorCmd &act_cmd,
                std::function<void(const Actuator::Feedback&)> feedback_cb,
                std::function<void(const GoalHandleActuator::WrappedResult&)> result_cb);


        bool is_awaiting_result();


    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<Actuator>::SharedPtr action_client_;

        //Status for blocking functionality
        ACTION_STATUS currentActionStatus;
};

#endif