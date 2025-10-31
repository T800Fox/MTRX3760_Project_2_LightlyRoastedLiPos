#include "mtrx3760_lrl_warehouse_inspection/inspectWarehouseController_node.hpp"


inspectWarehouseController::inspectWarehouseController()
: Node("inspect_warehouse_controller_node"){

    using namespace std::placeholders;

  //---Warenouse Inspection Server Setup---
  auto handle_goal = [this] (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const InspectWarehouse::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request -> UUID = %s", rclcpp_action::to_string(uuid).c_str());

    //Confirm that inspection can be performed (inspection not already in progress)
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT;
  }

  auto handle_cancel = [this] (const std::shared_ptr<InspectWarehouseActuator> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto handle_accepted = [this] (const std::shared_ptr<InspectWarehouseActuator> goal_handle)
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



    //This will be the update caller - will pass data into mazesolver cpp class 
    wall_dist_sub_ = this->create_subscription<warehouse_inspection::msg::WallDist>(
    "wall_dist", 
    qos, 
    std::bind(
        &mazeNavigator::wall_dist_callback,   
        this, 
        std::placeholders::_1));



  curr_pose_sub_ = this->create_subscription<warehouse_inspection::msg::Pose>(
    "curr_pose", 
    qos, 
    [this](const warehouse_inspection::msg::Pose::SharedPtr msg) {
        curr_pose.pos.x = msg->x;
        curr_pose.pos.y = msg->y;
        curr_pose.theta = msg->theta;
    }
  );


  wall_follower_logic = WallFollower();


  //Action client for actuator node ---------------
  this->actuator_client_ = rclcpp_action::create_client<Actuator>(
        this,
        "actuator");

  }


void inspectWarehouseController::wall_dist_callback(const warehouse_inspection::msg::WallDist::SharedPtr msg){

  auto send_goal_options = rclcpp_action::Client<Actuator>::SendGoalOptions();
    
    RCLCPP_INFO(this->get_logger(), "Options Initialised");

    send_goal_options.goal_response_callback = [this](const GoalHandleActuator::SharedPtr goal_handle)
    {
      if (!goal_handle.get()) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    RCLCPP_INFO(this->get_logger(), "Response Callback Defined");

    send_goal_options.result_callback = [this](const GoalHandleActuator::WrappedResult & result)
    {
      switch (result.code) 
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "LIGoal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Result Recieved -> success : %d", result.result->success);
    };

    //Pass lidar data and odom data into 

    auto act_cmd = wall_follower_logic.determine_cmd(lidar);
    this->client_ptr_->async_send_goal(act_cmd, send_goal_options);

    
    //Calculate refinement from wall - only if driving forwards
    if (act_cmd.mode = act_cmd.MODE_VEL_LINEAR){
      auto act_cmd = wall_follower_logic.calc_refinement(lidar);
      this->client_ptr_->async_send_goal(act_cmd, send_goal_options);
    }


    //Call action and block while awaiting success

}



