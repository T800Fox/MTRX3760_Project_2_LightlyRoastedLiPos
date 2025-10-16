#include "mtrx3760_oogway_mazeSolver/wallLocator_node.hpp"

using namespace std::chrono_literals;

wallLocator::wallLocator()
: Node("oogway_mazeSolver_node")
{
  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  wall_dist_pub_ = this->create_publisher<mtrx3760_oogway_mazesolver::msg::WallDist>("wall_dist", qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &wallLocator::scan_callback, \
      this, \
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Oogway Wall Locator node has been initialised");
}

wallLocator::~wallLocator()
{
  RCLCPP_INFO(this->get_logger(), "Oogway Wall Locator node has been terminated");
}

void wallLocator::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  //RCLCPP_INFO(this->get_logger(), "scan callback fired");  // STUB
  wall_dist_publisher(msg);
}

void wallLocator::wall_dist_publisher(const sensor_msgs::msg::LaserScan::SharedPtr data)
{

  // publish
  mtrx3760_oogway_mazesolver::msg::WallDist wall_dist;

  for (int i=0; i<4; i++){
    //For burger, LiDar readings are at almost exactly 1 degree increments
    wall_dist.distance[i] = data->ranges.at(i*90);
    wall_dist.wall_pres[i] = wall_dist.distance[i] < ( i%2 ? DIST_THRESHOLD_S : DIST_THRESHOLD_F ) 
                            && wall_dist.distance[i] != 0.0; //Inf/oor distance is expressed as 0.0
  } 

  wall_dist_pub_->publish(wall_dist);
  

}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wallLocator>());
  rclcpp::shutdown();

  return 0;
}