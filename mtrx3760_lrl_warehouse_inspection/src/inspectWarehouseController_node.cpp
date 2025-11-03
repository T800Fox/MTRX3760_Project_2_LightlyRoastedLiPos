#include "mtrx3760_lrl_warehouse_inspection/inspectWarehouseController_node.hpp"


inspectWarehouseController::inspectWarehouseController()
: Node("inspect_warehouse_controller_node"), actuator_client_wrapper_() 
{
  using namespace std::placeholders;  

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



      wall_follower_logic = wallFollower();
      
      //Init actuator wrapper
      auto node_shared_ptr = std::static_pointer_cast<rclcpp::Node>(this->shared_from_this());
      actuator_client_wrapper_ = ActuatorClientWrapper(node_shared_ptr, "actuator");


      //Create refined odom topic:

      refined_pose_pub_ = create_publisher<mtrx3760_lrl_warehouse_inspection::msg::Pose>("refined_pose", 10);
      
      offset_x = offset_y = 0.0;
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", qos, 
        [this](const nav_msgs::msg::Odometry::SharedPtr msg){

          //Calculate yaw
          double x = msg->pose.pose.orientation.x;
          double y = msg->pose.pose.orientation.y;
          double z = msg->pose.pose.orientation.z;
          double w = msg->pose.pose.orientation.w;

          double siny_cosp = 2.0 * (w * z + x * y);
          double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
          
          double rot = 180/3.14159 * std::atan2(siny_cosp, cosy_cosp);


          curr_pose.pos.x = msg->pose.pose.position.x;
          curr_pose.pos.y = msg->pose.pose.position.y;
          curr_pose.theta = rot;


          refined_pose.pos.x = curr_pose.pos.x + offset_x;
          refined_pose.pos.y = curr_pose.pos.y + offset_y;
          //refined_pose.theta = rot + rot_offset;
          
          /*
          auto refined_pose_msg = mtrx3760_lrl_warehouse_inspection::msg::Pose();
          refined_pose_msg.x = refined_pose.pos.x; curr_pose.pos.x
          refined_pose_msg.y = refined_pose.pos.y; curr_pose.pos.y
          refined_pose_msg.theta = rot;
          */

          auto refined_pose_msg = mtrx3760_lrl_warehouse_inspection::msg::Pose();
          refined_pose_msg.x = curr_pose.pos.x;
          refined_pose_msg.y = curr_pose.pos.y;
          refined_pose_msg.theta = rot;

          
          refined_pose_pub_->publish(refined_pose_msg);
          RCLCPP_INFO(this->get_logger(), "refined odom!");
        }
      );

      RCLCPP_INFO(this->get_logger(), "Completed Constructor");
      this->delayed_wrapper_init_timer_->cancel();

    }
  );
}


inspectWarehouseController::~inspectWarehouseController(){}

void inspectWarehouseController::wall_dist_callback(const mtrx3760_lrl_warehouse_inspection::msg::WallDist::SharedPtr msg){
  //Pass lidar data and odom data into 
  if (!actuator_client_wrapper_.is_accepting_goals()){return;}


  std::string debug_buf;

  //Generate actuator goals
  const static double dt = 0.1;

  auto act_cmd = wall_follower_logic.determine_cmd(msg->wall_pres, msg->distance, debug_buf);
  auto refine_cmd = wall_follower_logic.calc_refinement(msg->distance, debug_buf);
  RCLCPP_INFO(this->get_logger(), "DEBUG OUTPUT:::::  %s", debug_buf.c_str());

  //Omnly when refining!!!!
  if (act_cmd.mode == act_cmd.MODE_VEL_LINEAR){

    offset_x += sin(curr_pose.theta * 3.14159/180) * wall_follower_logic.query_roc();
    offset_y += cos(curr_pose.theta * 3.14159/180) * wall_follower_logic.query_roc();

    RCLCPP_INFO(this->get_logger(), "Applying offset");

  }

  //RCLCPP_INFO(this->get_logger(), "Logic debug: %s", debug_buf.c_str());

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