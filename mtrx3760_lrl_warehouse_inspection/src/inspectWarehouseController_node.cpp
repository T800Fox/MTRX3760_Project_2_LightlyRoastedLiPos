#include "mtrx3760_lrl_warehouse_inspection/inspectWarehouseController_node.hpp"


inspectWarehouseController::inspectWarehouseController()
: Node("inspect_warehouse_controller_node"), actuator_client_wrapper_() 
{
  using namespace std::placeholders;

  test_client_ptr_ = rclcpp_action::create_client<Actuator>(this, "actuator");

  

/*
  //---Warenouse Inspection Server Setup---
  auto handle_goal = [this] (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const InspectWarehouse::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request -> UUID = %s", rclcpp_action::to_string(uuid).c_str());

    //Confirm that inspection can be performed (inspection not already in progress)
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT;
  }

  auto handle_cancel = [this] (const std::shared_ptr<GoalHandleInspectWarehouse> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto handle_accepted = [this] (const std::shared_ptr<GoalHandleInspectWarehouse> goal_handle)
  {
    //Perform warehouse inspection. Start up cv node and call wall-follower routine
  };

  this->inspection_action_server_ = rclcpp_action::create_server<InspectWarehouse>(
    this,
    "inspect_warehouse",
    handle_goal,
    handle_cancel,
    handle_accepted
  );
*/


  this->delayed_wrapper_init_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    [this](){

      auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

      //This will be the update caller - will pass data into logic class 
      wall_dist_sub_ = this->create_subscription<mtrx3760_lrl_warehouse_inspection::msg::WallDist>(
      "wall_dist", 
      qos, 
      std::bind(
          &inspectWarehouseController::wall_dist_callback,   
          this, 
          std::placeholders::_1));


      curr_pose_sub_ = this->create_subscription<mtrx3760_lrl_warehouse_inspection::msg::Pose>(
        "curr_pose", 
        qos, 
        [this](const mtrx3760_lrl_warehouse_inspection::msg::Pose::SharedPtr msg) {
            curr_pose.pos.x = msg->x;
            curr_pose.pos.y = msg->y;
            curr_pose.theta = msg->theta;
        }
      );

      wall_follower_logic = wallFollower();
      
      init_wrapper(); 

      RCLCPP_INFO(this->get_logger(), "Completed Constructor");
    }
  );
}


void inspectWarehouseController::init_wrapper()
{
  this->delayed_wrapper_init_timer_->cancel();

  auto node_shared_ptr = std::static_pointer_cast<rclcpp::Node>(this->shared_from_this());
  actuator_client_wrapper_ = ActuatorClientWrapper(node_shared_ptr, "actuator");
}

inspectWarehouseController::~inspectWarehouseController(){}

void inspectWarehouseController::wall_dist_callback(const mtrx3760_lrl_warehouse_inspection::msg::WallDist::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "Wall Dist Callback Fired");

  //Pass lidar data and odom data into 
  if (actuator_client_wrapper_.is_awaiting_result()){return;}

  //Generate actuator goals
  auto act_cmd = wall_follower_logic.determine_cmd(msg->wall_pres, msg->distance);
  auto refine_cmd = wall_follower_logic.calc_refinement(msg->distance);



  //Send main action goal with refinement-goal chained to result callback (only when in linear-vel mode)
  actuator_client_wrapper_.send_goal(act_cmd, nullptr,
    (act_cmd.mode == act_cmd.MODE_VEL_LINEAR) ?
    [this, &refine_cmd](const GoalHandleActuator::WrappedResult&){
      actuator_client_wrapper_.send_goal(refine_cmd, nullptr, nullptr);
    } 
    : std::function<void(const GoalHandleActuator::WrappedResult &)>{}
  );


}



// Spin
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<inspectWarehouseController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}