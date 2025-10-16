#include "mtrx3760_oogway_mazeSolver/goalChecker_node.hpp"


using namespace std::chrono_literals;

goalChecker::goalChecker()
: Node("oogway_goalChecker_node")
{
  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  at_goal_pub_ = this->create_publisher<std_msgs::msg::Bool>("at_goal", qos); 

  // Initialise subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &goalChecker::odom_callback, \
      this, \
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Oogway Goal Checker node has been initialised");
}

goalChecker::~goalChecker()
{
  RCLCPP_INFO(this->get_logger(), "Oogway Goal Checker simulation node has been terminated");
}

void goalChecker::odom_callback(const nav_msgs::msg::Odometry msg)
{
  //RCLCPP_INFO(this->get_logger(), "odom callback fired");  // STUB
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<goalChecker>());
  rclcpp::shutdown();

  return 0;
}