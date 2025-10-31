#include "mtrx3760_lrl_actuator/actuator_debug_client_node.hpp"

mtrx3760_lrl_warehousebot::ActuatorDebugClient::ActuatorDebugClient() : Node("actuator_debug_client")
{
    this->client_ptr_ = rclcpp_action::create_client<Actuator>(
        this,
        "actuator");

    auto timer_callback_lambda = [this](){ return this->send_goal(); };

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        timer_callback_lambda);
}

void mtrx3760_lrl_warehousebot::ActuatorDebugClient::send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Actuator::Goal();
    goal_msg.mode = goal_msg.MODE_ABS_ANGULAR;
    goal_msg.magnitude = 3.14/2.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Actuator>::SendGoalOptions();
    
    send_goal_options.goal_response_callback = [this](const GoalHandleActuator::SharedPtr goal_handle)
    {
      if (!goal_handle.get()) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](GoalHandleActuator::SharedPtr, const std::shared_ptr<const Actuator::Feedback> feedback)
    {
      RCLCPP_INFO(this->get_logger(), "Made Feedback Call -> (err_n, err_t) : (%.2f, %.2f)", feedback->err_t, feedback->err_n);

    };

    send_goal_options.result_callback = [this](const GoalHandleActuator::WrappedResult & result)
    {
      switch (result.code) 
      {
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

      // std::stringstream ss;
      // ss << "Result received: ";
      // for (auto number : result.result->sequence) {
      //   ss << number << " ";
      // }
      // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
      RCLCPP_INFO(this->get_logger(), "Result Recieved -> success : %d", result.result->success);
      rclcpp::shutdown();
    };
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<mtrx3760_lrl_warehousebot::ActuatorDebugClient>();
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  
  return 0;
}