#include "mtrx3760_lrl_actuator_test_client/polling_action_client_node.hpp"



AddisonActuatorActionClient::AddisonActuatorActionClient(const rclcpp::NodeOptions& options)
  : Node("timed_rotation_action_client_cpp", options)
{
  // Create the action client
  action_client_ = rclcpp_action::create_client<Actuator>(
    this,
    "actuator"
  );

  currentActionStatus = UNDEFINED;

  std::vector<actionCommand> squareRoutine = {
    {0, 0.3},
    {1, 1.59},
    {0, 0.3},
    {1, 1.59},
    {0, 0.3},
    {1, 1.59},
    {0, 0.3},
    {1, 1.59}
  };

  addCommand({0, 0.3});
  addCommand({1, 1.59});
  addCommand({0, 0.3});
  addCommand({1, 1.59});
  addCommand({0, 0.3});
  addCommand({1, 1.59});
  addCommand({0, 0.3});
  addCommand({1, 1.59});



  send_goal();
}


void AddisonActuatorActionClient::goal_response_callback(const GoalHandleActuator::SharedPtr& goal_handle)
{
  if (!goal_handle)
  {
    currentActionStatus = UNDEFINED;
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
  }
  else
  {
    currentActionStatus = AWAITING_RESULT;
    RCLCPP_INFO(get_logger(), "Goal accepted by server");
    goal_handle_ = goal_handle;
  }

  return;
}

// void feedback_callback(
//   const GoalHandleTimedRotation::SharedPtr& goal_handle,
//   const std::shared_ptr<const TimedRotation::Feedback>& feedback)
// {
//   (void)goal_handle;  // Unused parameter
//   RCLCPP_INFO(get_logger(), "Feedback received - Elapsed time: %.2f seconds, Status: %s",
//               feedback->elapsed_time,
//               feedback->status.c_str());
// }

void AddisonActuatorActionClient::get_result_callback(const GoalHandleActuator::WrappedResult& result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Goal succeeded!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      return;
  }


  if(commandList.size() > 0)
  {
    send_goal();
  }
}

void AddisonActuatorActionClient::cancel_goal()
{
  RCLCPP_INFO(get_logger(), "Requesting to cancel goal");
  if (!goal_handle_)
  {
      RCLCPP_ERROR(get_logger(), "Goal handle is null");
      return;
  }

  auto future = action_client_->async_cancel_goal(goal_handle_);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
      RCLCPP_ERROR(get_logger(), "Failed to cancel goal");
      return;
  }

  RCLCPP_INFO(get_logger(), "Goal cancellation request sent");
}

void AddisonActuatorActionClient::send_goal() // int aMode, double aMagnitude
{
  RCLCPP_INFO(get_logger(), "Waiting for action server...");
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(get_logger(), "Action server not available after 5 seconds");
    return;
  }


  actionCommand currentCommand = fetchNextCommand();

  auto goal_msg = Actuator::Goal();
  goal_msg.mode = currentCommand.mode;
  goal_msg.magnitude = currentCommand.magnitude;

  RCLCPP_INFO(get_logger(),
              "Sending goal: mode=%d, magnitude=%.2f rad",
              goal_msg.mode, goal_msg.magnitude);

  auto send_goal_options = rclcpp_action::Client<Actuator>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&AddisonActuatorActionClient::goal_response_callback, this, std::placeholders::_1);

  // send_goal_options.feedback_callback =
  //   std::bind(&AddisonActuatorActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback =
    std::bind(&AddisonActuatorActionClient::get_result_callback, this, std::placeholders::_1);


  currentActionStatus = AWAITING_RESPONSE;
  action_client_->async_send_goal(goal_msg, send_goal_options);
}

void AddisonActuatorActionClient::addCommand(actionCommand aCommand)
{
  commandList.insert(commandList.begin(), aCommand);
}

actionCommand AddisonActuatorActionClient::fetchNextCommand()
{
  actionCommand outputCommand;

  if (commandList.size() > 0)
  {
    outputCommand = commandList.back();
    commandList.pop_back();
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Fetching Command When List Empty!");
    outputCommand.mode = 999;
    outputCommand.magnitude = 0;
  }

  return outputCommand;
}





int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<AddisonActuatorActionClient>(rclcpp::NodeOptions());
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}


// mtrx3760_lrl_warehousebot::ActuatorDebugClient::ActuatorDebugClient() : Node("actuator_debug_client")
// {
//   this->client_ptr_ = rclcpp_action::create_client<Actuator>(
//       this,
//       "actuator");

//   // auto timer_callback_lambda = [this](){ return this->sendGoalWPolling(); };

//   // this->timer_ = this->create_wall_timer(
//   //     std::chrono::milliseconds(500),
//   //     timer_callback_lambda);

//   currentActionStatus = UNDEFINED;

//   std::vector<actionCommand> squareRoutine = {
//     {0, 0.3},
//     {1, 1.59},
//     {0, 0.3},
//     {1, 1.59},
//     {0, 0.3},
//     {1, 1.59},
//     {0, 0.3},
//     {1, 1.59}
//   };

//   feedActuatorActions(squareRoutine);

// }

// void mtrx3760_lrl_warehousebot::ActuatorDebugClient::sendGoalWPolling(int aControlMethod, double aMagnitude)
// {
//     using namespace std::placeholders;

//     if (!this->client_ptr_->wait_for_action_server()) {
//       RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
//       rclcpp::shutdown();
//     }

//     // vaild command? , gonna assume magnitude always fine
//     if (aControlMethod != ABS_LINEAR && 
//         aControlMethod != ABS_ANGULAR &&
//         aControlMethod != VEL_LINEAR &&
//         aControlMethod != VEL_ANGULAR)
//     {
//       RCLCPP_ERROR(this->get_logger(), "Invaild control method -> %d", aControlMethod);
//       rclcpp::shutdown();
//     }

    
//     auto goal_msg = Actuator::Goal();
//     goal_msg.mode = aControlMethod;
//     goal_msg.magnitude = aMagnitude;

//     auto send_goal_options = rclcpp_action::Client<Actuator>::SendGoalOptions();
    
//     send_goal_options.goal_response_callback = [this](const GoalHandleActuator::SharedPtr goal_handle)
//     {
//       RCLCPP_INFO(this->get_logger(), "   response callback fired");
//       if (!goal_handle.get()) 
//       {
//         RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//         currentActionStatus = UNDEFINED;
//       } 
//       else 
//       {
//         RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
//         currentActionStatus = AWAITING_RESULT;
//       }
//     };

//     send_goal_options.feedback_callback = [this](GoalHandleActuator::SharedPtr, const std::shared_ptr<const Actuator::Feedback> feedback)
//     {
//       RCLCPP_INFO(this->get_logger(), "Made Feedback Call -> (err_n, err_t) : (%.2f, %.2f)", feedback->err_t, feedback->err_n);

//     };

//     send_goal_options.result_callback = [this](const GoalHandleActuator::WrappedResult & result)
//     {
      
//       switch (result.code) 
//       {
//         case rclcpp_action::ResultCode::SUCCEEDED:
//           break;
//         case rclcpp_action::ResultCode::ABORTED:
//           RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//           return;
//         case rclcpp_action::ResultCode::CANCELED:
//           RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//           return;
//         default:
//           RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//           return;
//       }

//       RCLCPP_INFO(this->get_logger(), "Result Recieved -> success : %d", result.result->success);

//       currentActionStatus = UNDEFINED;
//     };

//     RCLCPP_INFO(this->get_logger(), "Sending goal");
//     currentActionStatus = AWAITING_RESPONSE;
//     this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
// }

// void mtrx3760_lrl_warehousebot::ActuatorDebugClient::feedActuatorActions(std::vector<actionCommand> aRoutine)
// {
//   for(auto cmd : aRoutine)
//   {
//     RCLCPP_INFO(this->get_logger(), "Running Command -> mode : %d , magnitude: %.2f", cmd.mode, cmd.magnitude);
//     sendGoalWPolling(cmd.mode, cmd.magnitude);
//     // std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     while(currentActionStatus != UNDEFINED)
//     {
//       // std::this_thread::sleep_for(std::chrono::milliseconds(500));
//       RCLCPP_INFO(this->get_logger(), "Current State -> %d", currentActionStatus);
//     }
//   }
// }

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto action_client = std::make_shared<mtrx3760_lrl_warehousebot::ActuatorDebugClient>();
//   rclcpp::spin(action_client);
//   rclcpp::shutdown();
  
//   return 0;
// }







// send goal from boilerplate for actuator

// void mtrx3760_lrl_warehousebot::ActuatorDebugClient::send_goal()
//   {
//     using namespace std::placeholders;

//     this->timer_->cancel();

//     if (!this->client_ptr_->wait_for_action_server()) {
//       RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
//       rclcpp::shutdown();
//     }

//     auto goal_msg = Actuator::Goal();
//     goal_msg.mode = goal_msg.MODE_ABS_ANGULAR;
//     goal_msg.magnitude = 3.14/2.0;

//     RCLCPP_INFO(this->get_logger(), "Sending goal");

//     auto send_goal_options = rclcpp_action::Client<Actuator>::SendGoalOptions();
    
//     send_goal_options.goal_response_callback = [this](const GoalHandleActuator::SharedPtr goal_handle)
//     {
//       if (!goal_handle.get()) {
//         RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//       } else {
//         RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
//       }
//     };

//     send_goal_options.feedback_callback = [this](GoalHandleActuator::SharedPtr, const std::shared_ptr<const Actuator::Feedback> feedback)
//     {
//       RCLCPP_INFO(this->get_logger(), "Made Feedback Call -> (err_n, err_t) : (%.2f, %.2f)", feedback->err_t, feedback->err_n);

//     };

//     send_goal_options.result_callback = [this](const GoalHandleActuator::WrappedResult & result)
//     {
//       switch (result.code) 
//       {
//         case rclcpp_action::ResultCode::SUCCEEDED:
//           break;
//         case rclcpp_action::ResultCode::ABORTED:
//           RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//           return;
//         case rclcpp_action::ResultCode::CANCELED:
//           RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//           return;
//         default:
//           RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//           return;
//       }

//       // std::stringstream ss;
//       // ss << "Result received: ";
//       // for (auto number : result.result->sequence) {
//       //   ss << number << " ";
//       // }
//       // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
//       RCLCPP_INFO(this->get_logger(), "Result Recieved -> success : %d", result.result->success);
//       rclcpp::shutdown();
//     };
//     this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
//   }