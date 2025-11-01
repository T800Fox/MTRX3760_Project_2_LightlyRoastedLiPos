#include "mtrx3760_lrl_actuator/actuator_action_server_node.hpp"

//------------------------------------------------------------------------------------------
mtrx3760_lrl_warehousebot::ActuatorActionServer::ActuatorActionServer() : Node("actuator_action_server")
{
  using namespace std::placeholders;

  //---Action Server Setup---
  auto handle_goal = [this] (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Actuator::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request -> (mode, ang_mag, lin_mag) -> (%d , %.2f)", goal->mode, goal->magnitude);
    (void)uuid;

    
    // couldn't figure out what sort of variable could store the response codes; hence two return points
    if (manager.controllerRunning())
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    else
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  };

  auto handle_cancel = [this] (const std::shared_ptr<GoalHandleActuator> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto handle_accepted = [this] (const std::shared_ptr<GoalHandleActuator> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor,
    // so we declare a lambda function to be called inside a new thread
    auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
    std::thread{execute_in_thread}.detach();
  };

  this->action_server_ = rclcpp_action::create_server<Actuator>(
    this,
    "actuator",
    handle_goal,
    handle_cancel,
    handle_accepted);


  //---tf2 listener setup---
  buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
  listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
  timer = create_wall_timer(std::chrono::milliseconds(100),
  std::bind(&mtrx3760_lrl_warehousebot::ActuatorActionServer::listen_tf , this));


  //---cmd_vel publisher---
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", qos);
}
//---------------------------------------------
void mtrx3760_lrl_warehousebot::ActuatorActionServer::execute(const std::shared_ptr<GoalHandleActuator> goal_handle) 
{
  RCLCPP_INFO(this->get_logger(), "Executing goal...");

  if (goal_handle->get_goal()->mode == goal_handle->get_goal()->MODE_VEL_LINEAR)
  {
    RCLCPP_INFO(this->get_logger(), "Linear Velocity Command");

    geometry_msgs::msg::TwistStamped new_cmd_ = manager.updatePassiveVelLinear(goal_handle);
    cmd_vel_pub_->publish(new_cmd_);
  }
  else if (goal_handle->get_goal()->mode == goal_handle->get_goal()->MODE_VEL_ANGULAR)
  {
    RCLCPP_INFO(this->get_logger(), "Angular Velocity Command");

    geometry_msgs::msg::TwistStamped new_cmd_ = manager.updatePassiveVelAngular(goal_handle);
    cmd_vel_pub_->publish(new_cmd_);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Controller Command");

    manager.fireUpController(goal_handle);
    listen_tf();
  }
};
//---------------------------------------------
void mtrx3760_lrl_warehousebot::ActuatorActionServer::listen_tf() 
{
  try 
  {
    geometry_msgs::msg::TransformStamped transformStamped = buffer->lookupTransform("base_footprint", "odom", tf2::TimePointZero); // child, parent
     
    RCLCPP_INFO(this->get_logger(), "Making TF Available to Controllers");
    geometry_msgs::msg::TwistStamped new_cmd_ =  manager.routeTf(transformStamped);
    cmd_vel_pub_->publish(new_cmd_);
  } 
  catch (const tf2::TransformException & ex) 
  {
    RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
  }
}
//------------------------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<mtrx3760_lrl_warehousebot::ActuatorActionServer>();
  rclcpp::spin(action_server);
  rclcpp::shutdown();
  
  return 0;
}
//------------------------------------------------------------------------------------------