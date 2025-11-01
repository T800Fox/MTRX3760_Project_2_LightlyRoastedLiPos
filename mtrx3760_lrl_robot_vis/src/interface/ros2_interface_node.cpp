#include "ros2_interface_node.hpp"

ros_data shared_data; 


Ros2Interface::Ros2Interface()
: Node("ros2_interface_node")
{
  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

 
  interface_socket.create_server(8080);


  // Initialise subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &Ros2Interface::odom_callback, \
      this, \
      std::placeholders::_1));


  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&Ros2Interface::map_callback, this, std::placeholders::_1)
  );

  package_detection_sub_ = this->create_subscription<custom_interfaces::msg::PackageDetection>(
      "/package_detection", 10,
      std::bind(&Ros2Interface::package_detection_callback, this, std::placeholders::_1)
  );

  wall_follower_seg_sub_ = this->create_subscription<custom_interfaces::msg::LineSeg>(
      "/wall_follower_seg", 10,
      std::bind(&Ros2Interface::wall_follower_seg_callback, this, std::placeholders::_1)
  );


  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10,
      std::bind(&Ros2Interface::image_callback, this, std::placeholders::_1)
  );

  

      
  //Initialise update timer
  update_timer = this->create_wall_timer(
            100ms,
            std::bind(&Ros2Interface::update_callback, this)
  );


  RCLCPP_INFO(this->get_logger(), "Ros interface node initialised!");
}


Ros2Interface::~Ros2Interface(){}


void Ros2Interface::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){

  //Convert from Quaternions
  double x = msg->pose.pose.orientation.x;
  double y = msg->pose.pose.orientation.y;
  double z = msg->pose.pose.orientation.z;
  double w = msg->pose.pose.orientation.w;

  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  double rot = 180/3.14159 * std::atan2(siny_cosp, cosy_cosp);

  //Construct json packet
  json j = json{
      {"type", "data_pose"},
      {"x", (float) msg->pose.pose.position.x * 1000},
      {"y", (float) msg->pose.pose.position.y * 1000},
      {"rot", -rot}
  };
  
  interface_socket.send_json(j);

   //   RCLCPP_INFO(this->get_logger(), "Odom callback!");

}


void Ros2Interface::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  //Construct json packet
  json j = json{
    {"type", "data_map"},
    {"width", msg->info.width},
    {"height", msg->info.height},
    {"origin_x", msg->info.origin.position.x * 1000},
    {"origin_y", msg->info.origin.position.y * 1000},
    {"data", msg->data}
  };

  interface_socket.send_json(j);

  RCLCPP_INFO(this->get_logger(), "Map received: %dx%d Origin: %.2f, %.2f", msg->info.width, msg->info.height, msg->info.origin.position.x, msg->info.origin.position.y);
}


void Ros2Interface::package_detection_callback(const custom_interfaces::msg::PackageDetection::SharedPtr msg){
  //Construct json packet
  json j = json{
    {"type", "data_package_detection"},
    {"id", msg->id},
    {"pos_x", msg->global_pos.x * 1000},
    {"pos_y", msg->global_pos.y * 1000},
    {"var", msg->confidence},
    {"observation_count", msg->observation_count},
  };
  interface_socket.send_json(j);

  RCLCPP_INFO(this->get_logger(), "Package detection: ID %d", msg->id);

}



void Ros2Interface::image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
  std::vector<char> encoded_buf(((msg->data.size() + 2) / 3) * 4 + 1);
  hv_base64_encode(msg->data.data(), msg->data.size(), encoded_buf.data());
  std::string encoded(encoded_buf.data());

  json j = json{
    {"type", "data_image"},
    {"image_data", encoded},
    {"width", msg->width},
    {"height", msg->height},
    {"step", msg->step}
  };

  interface_socket.send_json(j);

  RCLCPP_INFO(this->get_logger(), "Image");
}






void Ros2Interface::wall_follower_seg_callback(const custom_interfaces::msg::LineSeg::SharedPtr msg){
    //Construct json packet
  json j = json{
    {"type", "data_wall_follower_seg"},
    {"start_pos_x", msg->start_pos.x * 1000},
    {"start_pos_y", msg->start_pos.y * 1000},
    {"end_pos_x", msg->end_pos.x * 1000},
    {"end_pos_y", msg->end_pos.y * 1000},
  };
  interface_socket.send_json(j);

  RCLCPP_INFO(this->get_logger(), "Wall follower line-seg");
}


void Ros2Interface::update_callback(){
  json packet;

  packet = interface_socket.query_prev_packet("srv_inspect_warehouse", true);
  if (!packet.empty()){
    //Inspect warehouse request was recieved
    RCLCPP_INFO(this->get_logger(), "Inspect warehouse!");
  }

  packet = interface_socket.query_prev_packet("srv_perform_delivery", true);
  if (!packet.empty()){
    //Perform delivery request was recieevd
    std::vector<PackRequest> parsed_requests = packet["package_requests"].get<std::vector<PackRequest>>();
    SolverParams solver_params = packet["solver_params"].get<SolverParams>();

    RCLCPP_INFO(this->get_logger(), "Perform delivery:");
  }

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