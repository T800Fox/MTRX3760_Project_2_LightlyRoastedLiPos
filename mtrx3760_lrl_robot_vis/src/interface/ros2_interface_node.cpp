#include "mtrx3760_lrl_robot_vis/ros2_interface_node.hpp"



Ros2Interface::Ros2Interface()
: Node("ros2_interface_node")
{
  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

 
  interface_socket.create_server(8080);


  // Initialise subscribers
  refined_pose_sub_ = this->create_subscription<mtrx3760_lrl_warehouse_inspection::msg::Pose>(
    "/refined_pose", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &Ros2Interface::refined_pose_callback, \
      this, \
      std::placeholders::_1));


  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&Ros2Interface::map_callback, this, std::placeholders::_1)
  );

  package_detection_sub_ = this->create_subscription<MarkerDetection>(
      "/detections/raw", 10,
      std::bind(&Ros2Interface::package_detection_callback, this, std::placeholders::_1)
  );

  wall_follower_seg_sub_ = this->create_subscription<mtrx3760_lrl_interfaces::msg::LineSeg>(
      "/wall_follower_seg", 10,
      std::bind(&Ros2Interface::wall_follower_seg_callback, this, std::placeholders::_1)
  );

   battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10,
      std::bind(&Ros2Interface::battery_callback, this, std::placeholders::_1)
  );


  

      
  //Initialise update timer
  update_timer = this->create_wall_timer(
            100ms,
            std::bind(&Ros2Interface::update_callback, this)
  );


  RCLCPP_INFO(this->get_logger(), "Ros interface node initialised!");
}


Ros2Interface::~Ros2Interface(){}


void Ros2Interface::refined_pose_callback(const mtrx3760_lrl_warehouse_inspection::msg::Pose::SharedPtr msg){
  //Construct json packet
  json j = json{
      {"type", "data_pose"},
      {"x", (float) msg->x * 1000},
      {"y", (float) msg->y * 1000},
      {"rot", -(float) msg->theta}
  };
  
  interface_socket.send_json(j);
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


void Ros2Interface::package_detection_callback(const MarkerDetection::SharedPtr msg){
  
  std::string encoded;
  if (msg->closest_image.data.size() && false){
    //Encode image
    std::vector<char> encoded_buf(((msg->closest_image.data.size() + 2) / 3) * 4 + 1);
    hv_base64_encode(msg->closest_image.data.data(), msg->closest_image.data.size(), encoded_buf.data());
    encoded = encoded_buf.data();
  }

  //RCLCPP_INFO(this->get_logger(), "Recieved package detection!!!!!");

  //Construct json packet
  json j = json{
    {"type", "data_package_detection"},
    {"id", msg->id},
    {"pos_x", msg->global_position.x * 1000},
    {"pos_y", msg->global_position.y * 1000},
    {"confidence_radius", msg->covariance_radius * 1000},
    {"observation_count", msg->observation_count},
    //{"image_data", encoded},
    {"image_width", msg->closest_image.width},
    {"image_height", msg->closest_image.height},
    {"image_step", msg->closest_image.step},
  };



  interface_socket.send_json(j);

  RCLCPP_INFO(this->get_logger(), "Package detection: ID %d", msg->id);

}


void Ros2Interface::wall_follower_seg_callback(const mtrx3760_lrl_interfaces::msg::LineSeg::SharedPtr msg){
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



void Ros2Interface::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg){
   json j = json{
      {"type", "data_battery"},
      {"perc", msg->percentage},
  };
  
  interface_socket.send_json(j);
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
    std::vector<PackRequest> package_requests = packet["package_requests"].get<std::vector<PackRequest>>();
    SolverParams solver_params = packet["solver_params"].get<SolverParams>();

    //CONSTRUCT ACTION GOAL AND SEND
    
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