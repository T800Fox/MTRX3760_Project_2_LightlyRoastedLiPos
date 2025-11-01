#include "client.hpp"


ActionClientWrapper::ActionClientWrapper(rclcpp::Node::SharedPtr node, const std::string &action_name)
    : node_(node) {
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name);
}

bool ActionClientWrapper::wait_for_server(std::chrono::seconds timeout) {
    return action_client_->wait_for_action_server(timeout);
}



bool ActionClientWrapper::is_awaiting_result(){
    return (currentActionStatus == AWAITING_RESULT);
}

void ActionClientWrapper::send_goal(const Goal &goal,
                std::function<void(const Feedback&)> feedback_cb,
                std::function<void(const Result&, rclcpp_action::ResultCode)> result_cb){

    wait_for_server();

    //Awaiting acceptance of goal request
    currentActionStatus = AWAITING_RESPONSE;

    rclcpp_action::Client<ActionT>::SendGoalOptions options;
    options.goal_response_callback = [this](const GoalHandleAction::SharedPtr& goal_handle) {
        if (!goal_handle){
            currentActionStatus = UNDEFINED;
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        }
        else{
            currentActionStatus = AWAITING_RESULT;
            RCLCPP_INFO(get_logger(), "Goal accepted by server");
            goal_handle_ = goal_handle;
        }

        return;
    };

    options.feedback_callback = [this, feedback_cb](GoalHandleAction::SharedPtr,
                                const std::shared_ptr<const ActionT::Feedback> feedback) {

        if (feedback_cb){
            feedback_cb(feedback);
        }
    };

    options.result_callback = [this, result_cb](const GoalHandleAction::WrappedResult& result) {
        //Update the status - free block
        currentActionStatus = UNDEFINED;

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        //Call result callback (if existent)
        if (result_cb){
            result_cb(result);
        }

    };

    action_client_->async_send_goal(goal, options);
}
