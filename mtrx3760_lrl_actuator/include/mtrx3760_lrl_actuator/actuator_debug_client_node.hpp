#ifndef MTRX3760_LRL_WAREHOUSEBOT_ACTUATOR_DEBUG_CLIENT_NODE_HPP_
#define MTRX3760_LRL_WAREHOUSEBOT_ACTUATOR_DEBUG_CLIENT_NODE_HPP_

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "mtrx3760_lrl_interfaces/action/test.hpp"

namespace mtrx3760_lrl_warehousebot
{
    class ActuatorDebugClient : public rclcpp::Node
    {


        public:
            using Actuator = mtrx3760_lrl_interfaces::action::Test;
            using GoalHandleActuator = rclcpp_action::ClientGoalHandle<Actuator>;

            ActuatorDebugClient();
            void send_goal();
        

        private:
            rclcpp_action::Client<Actuator>::SharedPtr client_ptr_;
            rclcpp::TimerBase::SharedPtr timer_;
    };
}  // namespace custom_action_cpp
#endif