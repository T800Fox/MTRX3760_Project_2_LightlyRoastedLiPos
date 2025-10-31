#include "ros2_interface_node.hpp"

ros_data shared_data; 


Ros2Interface::Ros2Interface()
: Node("ros2_interface_node")
{
  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &Ros2Interface::odom_callback, \
      this, \
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Ros visualiser node initialised!");
}


Ros2Interface::~Ros2Interface(){}

void Ros2Interface::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  shared_data.robot_pos = Point{(float) msg->pose.pose.position.x, (float) msg->pose.pose.position.y};

  //Convert from Quaternions
  double x = msg->pose.pose.orientation.x;
  double y = msg->pose.pose.orientation.y;
  double z = msg->pose.pose.orientation.z;
  double w = msg->pose.pose.orientation.w;

  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  shared_data.robot_rot = 180/3.14159 * std::atan2(siny_cosp, cosy_cosp);

  //RCLCPP_INFO(this->get_logger(), "Odom callback x: y: rot: %.2f, %.2f, %.2f %.2f", shared_data.robot_pos.x, shared_data.robot_pos.y, shared_data.robot_rot, msg->pose.pose.position.y);
}



/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2Interface>());
  rclcpp::shutdown();

  return 0;
}