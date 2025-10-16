#include "mtrx3760_oogway_mazeSolver/robotActuator_node.hpp"



using namespace std::chrono_literals;


robotActuator::robotActuator()
: Node("oogway_robotActuator_node")
{
  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", qos); 
  is_abs_rotating_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_abs_rotating", qos);
  is_abs_moving_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_abs_moving", qos); 
  curr_pose_pub_ = this->create_publisher<mtrx3760_oogway_mazesolver::msg::Pose>("curr_pose", qos); 


  // Initialise subscribers
  pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        qos,
        std::bind(
        &robotActuator::odom_callback,
        this,
        std::placeholders::_1)
  );

  angular_cmd_sub_ = this->create_subscription<mtrx3760_oogway_mazesolver::msg::AngularCmd>(
        "angular_cmd",
        qos,
        std::bind(
        &robotActuator::angular_cmd_callback,
        this,
        std::placeholders::_1)
  );

  linear_cmd_sub_ = this->create_subscription<mtrx3760_oogway_mazesolver::msg::LinearCmd>(
        "linear_cmd",
        qos,
        std::bind(
        &robotActuator::linear_cmd_callback,
        this,
        std::placeholders::_1)
  );


  // Initialise timers
  angle_feedback_timer_ = this->create_wall_timer(
            100ms,
            std::bind(
            &robotActuator::angle_feedback_callback,
            this)
  );

  linear_feedback_timer_ = this->create_wall_timer(
            100ms,
            std::bind(
            &robotActuator::linear_feedback_callback,
            this)
  );

  // Cancel till targets are provided
  angle_feedback_timer_->cancel(); 
  linear_feedback_timer_->cancel(); 

  // Initialise member variables
  last_is_abs_rotating = false;
  last_is_abs_moving = false;

  prev_ang_vel = 0.0;
  prev_lin_vel = 0.0;


  RCLCPP_INFO(this->get_logger(), "Oogway Maze Actuator node has been initialised");
}

robotActuator::~robotActuator()
{
  RCLCPP_INFO(this->get_logger(), "Oogway Maze Actuator simulation node has been terminated");
}



void robotActuator::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    curr_pose.pos.x = msg->pose.pose.position.x;
    curr_pose.pos.y = msg->pose.pose.position.y;

    // Convert quaternion to yaw (heading)
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    curr_pose.theta = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz)
    );

    //Publish abstracted pose from odom-data
    mtrx3760_oogway_mazesolver::msg::Pose pose;
    pose.x = curr_pose.pos.x;
    pose.y = curr_pose.pos.y;
    pose.theta = curr_pose.theta;
    curr_pose_pub_->publish(pose);
}


void robotActuator::angular_cmd_callback(const mtrx3760_oogway_mazesolver::msg::AngularCmd::SharedPtr msg){
  //Check command type (absolute or vel)


  if (msg->mode == msg->MODE_ABSOLUTE){
    //Update target angle and resume feedback loop (invoke callback instantly to update state)
    target_angle = msg->target_angle;
    starting_pose = curr_pose;

    angle_feedback_timer_->reset();
    angle_feedback_callback();

    RCLCPP_INFO(this->get_logger(), "ANGULAR COMMAND (ABS): Setting target to: %.2f", target_angle);

  } else if (msg->mode == msg->MODE_VELOCITY){
    //Cancel feedback-loop (if its active)
    angle_feedback_timer_->cancel();

    //Publish desired angular-vel
    double angular_vel = std::clamp(msg->angular_vel, -MAX_ANG_VEL, MAX_ANG_VEL);

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.twist.angular.z = angular_vel;
    cmd_vel.twist.linear.x = prev_lin_vel;
    cmd_vel_pub_->publish(cmd_vel);

    prev_ang_vel = angular_vel;

    RCLCPP_INFO(this->get_logger(), "ANGULAR COMMAND (VEL): Setting velocity to: %.2f", msg->angular_vel);
  
  } else{
    //Invalid command type
    RCLCPP_INFO(this->get_logger(), "ANGULAR COMMAND: invalid command type");

  }
  
}

void robotActuator::angle_feedback_callback(){
  geometry_msgs::msg::TwistStamped cmd_vel;
  std_msgs::msg::Bool is_abs_rotating;
  
  double angle_diff =  target_angle - curr_pose.theta;

  //Normalise error
  double error = std::atan2(std::sin(angle_diff), std::cos(angle_diff));


  //Check if target angle is within threshold of accuracy
  if (std::fabs(error) > ANGLE_ACCURACY_THRESH){
    cmd_vel.twist.angular.z = std::clamp(std::fabs(error) * 1.5,
                                          0.05, MAX_ANG_VEL) * (error > 0 ? 1 : -1 );

    is_abs_rotating.data = true;

    RCLCPP_INFO(this->get_logger(), "ANGULAR COMMAND: Curr angle: %.2f", curr_pose.theta * 180/M_PI);

  } else{
    //Target angle has been reached - disable the feedback timer untill a new target is provided
    cmd_vel.twist.angular.z = 0.0f;
    angle_feedback_timer_->cancel();
    is_abs_rotating.data = false;

    RCLCPP_INFO(this->get_logger(), "ANGULAR COMMAND: Rotation completed! target: %.2f curr: %.2f", target_angle * 180/M_PI, curr_pose.theta * 180/M_PI);
  }
  
  cmd_vel_pub_->publish(cmd_vel);

  //Only publish if update has occured
  if (is_abs_rotating.data != last_is_abs_rotating){
    is_abs_rotating_pub_->publish(is_abs_rotating);
    last_is_abs_rotating = is_abs_rotating.data;
  }
  
}


void robotActuator::linear_cmd_callback(const mtrx3760_oogway_mazesolver::msg::LinearCmd::SharedPtr msg){
  //Check command type (absolute or vel)
  if (msg->mode == msg->MODE_ABSOLUTE){
    //Update target angle and resume feedback loop (invoke callback instantly to update state)
    target_dist = msg->target_distance;
    starting_pose = curr_pose;

    linear_feedback_timer_->reset();
    linear_feedback_callback();

    RCLCPP_INFO(this->get_logger(), "LINEAR COMMAND (ABS): Setting target to: %.2f", target_dist);

  } else if (msg->mode == msg->MODE_VELOCITY){
    //Cancel feedback-loop (if its active)
    linear_feedback_timer_->cancel();

    //Publish desired angular-vel
    double lin_vel = std::clamp(msg->linear_vel, -MAX_LIN_VEL, MAX_LIN_VEL);
    
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.twist.linear.x = lin_vel;
    cmd_vel.twist.angular.z = prev_ang_vel;
    cmd_vel_pub_->publish(cmd_vel);

    prev_lin_vel = lin_vel;

    RCLCPP_INFO(this->get_logger(), "LINEAR COMMAND (VEL): Setting velocity to: %.2f", lin_vel);
  
  } else{
    //Invalid command type
    RCLCPP_INFO(this->get_logger(), "LINEAR COMMAND: invalid command type");

  }
  
}

void robotActuator::linear_feedback_callback(){
  geometry_msgs::msg::TwistStamped cmd_vel;
  std_msgs::msg::Bool is_abs_moving;
  
  double dist = dist2D(curr_pose.pos, starting_pose.pos);
  double error =  target_dist - dist;


  //Check if target dist is within threshold of accuracy
  if (std::fabs(error) > DIST_ACCURACY_THRESH){

    double progress = std::clamp(dist / target_dist, 0.01, 0.99);

    // Easing function: bell-shaped profile for smooth in/out
    double ease_shape = std::sin(progress * M_PI);  // 0 → 1 → 0

    cmd_vel.twist.linear.x = std::clamp(ease_shape * MAX_LIN_VEL,
                                          0.05, MAX_LIN_VEL);


    RCLCPP_INFO(this->get_logger(), "LINEAR COMMAND: Curr dist: %.2f", dist);                             

    is_abs_moving.data = true;

  } else{
    //Target dist has been reached - disable the feedback timer untill a new target is provided
    cmd_vel.twist.linear.x = 0.0f;
    linear_feedback_timer_->cancel();
    is_abs_moving.data = false;

    RCLCPP_INFO(this->get_logger(), "LINEAR COMMAND: move completed!!!! target dist: %.2f curr: %.2f", target_dist, dist);
  }
  
  //Publish velocity
  cmd_vel_pub_->publish(cmd_vel);

  // Publish state
  if (is_abs_moving.data != last_is_abs_moving){
    is_abs_moving_pub_->publish(is_abs_moving);
    last_is_abs_moving = is_abs_moving.data;
  }
  
}




/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotActuator>());
  rclcpp::shutdown();

  return 0;
}