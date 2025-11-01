#include "mtrx3760_lrl_actuator/actuator_client.hpp"


ActuatorClientWrapper::ActuatorClientWrapper(rclcpp::Node::SharedPtr node, const std::string &action_name)
    : node_(node) {
    action_client_ = rclcpp_action::create_client<Actuator>(node, action_name);
    
}


bool ActuatorClientWrapper::wait_for_server(std::chrono::seconds timeout) {
    return action_client_->wait_for_action_server(timeout);
}




bool ActuatorClientWrapper::is_awaiting_result(){
    return (currentActionStatus == AWAITING_RESULT);
}

void ActuatorClientWrapper::send_goal(ActuatorCmd &act_cmd,
                std::function<void(const Actuator::Feedback&)> feedback_cb,
                std::function<void(const GoalHandleActuator::WrappedResult&)> result_cb){

    wait_for_server();

    //Awaiting acceptance of goal request
    currentActionStatus = AWAITING_RESPONSE;

    rclcpp_action::Client<Actuator>::SendGoalOptions options;
    options.goal_response_callback = [this](const GoalHandleActuator::SharedPtr& goal_handle) {
        if (!goal_handle){
            currentActionStatus = UNDEFINED;
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        }
        else{
            currentActionStatus = AWAITING_RESULT;
            RCLCPP_INFO(node_->get_logger(), "Goal accepted by server");
         //   goal_handle_ = goal_handle;
        }

        return;
    };

    

    options.feedback_callback = [this, feedback_cb](GoalHandleActuator::SharedPtr,
                                const std::shared_ptr<const Actuator::Feedback> feedback) {

        if (feedback_cb){
            feedback_cb(*feedback);
        }
    };

    options.result_callback = [this, result_cb](const GoalHandleActuator::WrappedResult& result) {
        //Update the status - free block
        currentActionStatus = UNDEFINED;

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
                return;
        }
  

        //Call result callback (if existent)
        if (result_cb){
            result_cb(result);
        }

    };

    auto goal = Actuator::Goal();
    goal.mode = act_cmd.mode;
    goal.magnitude = act_cmd.magnitude;

    action_client_->async_send_goal(goal, options);
    
}
