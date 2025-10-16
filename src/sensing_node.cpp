#include "mtrx3760_LRL_warehouseBot/sensing_node.hpp"

using namespace std::chrono_literals;

sensing::sensing() : Node("lrl_sensing_node")
{
  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  heading_pub_ = this->create_publisher<std_msgs::msg::Float64>("heading", qos); 

  // Initialise subscribers
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &sensing::imu_callback, \
      this, \
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "LRL Sensing node has been initialised");
}

sensing::~sensing()
{
  RCLCPP_INFO(this->get_logger(), "LRL Sensing node has been terminated");
}

void sensing::imu_callback(const sensor_msgs::msg::Imu msg)
{
  // Convert quaternion to roll, pitch, yaw
  tf2::Quaternion tf_quat(
      msg.orientation.x,
      msg.orientation.y,
      msg.orientation.z,
      msg.orientation.w
  );

  tf2::Matrix3x3 m(tf_quat);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);


  // display data + publish heading
  RCLCPP_INFO(this->get_logger(), "IMU Data --> Roll: %.2f Pitch: %.2f Yaw: %.2f", roll, pitch, yaw);
  
  std_msgs::msg::Float64 outputMsg;
  outputMsg.data = yaw;
  heading_pub_->publish(outputMsg);
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensing>());
  rclcpp::shutdown();

  return 0;
}