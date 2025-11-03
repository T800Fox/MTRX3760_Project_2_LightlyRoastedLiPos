#include "mtrx3760_lrl_package_delivery/deliveryController_node.hpp"


DeliveryController::DeliveryController()
: Node("delivery_controller_node") // Constructor
{
    
    using namespace std::placeholders;


    is_accepting_goals = true;

    //---Warenouse Inspection Server Setup---
    auto handle_goal = [this] (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PerformDelivery::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request -> UUID = %s", rclcpp_action::to_string(uuid).c_str());

        (void)uuid;
        if (is_accepting_goals){
            RCLCPP_INFO(this->get_logger(), "Delivery is begining - ACCEPTED", rclcpp_action::to_string(uuid).c_str());
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

        } else{
            //Reject goal if already in progress
            RCLCPP_INFO(this->get_logger(), "Delivery already in progress - REJECTED", rclcpp_action::to_string(uuid).c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
    };

    auto handle_cancel = [this] (const std::shared_ptr<GoalHandlePerformDelivery> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [this] (const std::shared_ptr<GoalHandlePerformDelivery> goal_handle){
        //Kill map topic
        occupancy_grid_subscriber_.reset();

        //Execute
        using namespace std::placeholders;
        std::thread{std::bind(&DeliveryController::delivery_action_execute, this, _1), goal_handle}.detach();

    };

    

    this->delivery_action_server_ = rclcpp_action::create_server<PerformDelivery>(
        this,
        "perform_delivery",
        handle_goal,
        handle_cancel,
        handle_accepted
    );


    //--------------------ROS SUBSCRIBERS-------------------//
    // Subscriber for package data received from camera node
    package_detection_sub_ = this->create_subscription<mtrx3760_lrl_interfaces::msg::MarkerDetection>(
            "/detections/raw", 10,
            std::bind(&DeliveryController::package_detection_callback, this, std::placeholders::_1)
    );
        

    //REPLACE WITH TF AMCL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    robot_pose_sub_ = this->create_subscription<mtrx3760_lrl_interfaces::msg::Pose>(
        "/refined_pose", \
        rclcpp::SensorDataQoS(), \
        [this](const mtrx3760_lrl_interfaces::msg::Pose::SharedPtr msg){
            curr_pose = msg;
        }
    );

    //Occupancy grid subscriber
    occupancy_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>
    (
        "/map",
        10,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            most_recent_map = msg;
            RCLCPP_INFO(this->get_logger(), "Map recieved.");
        }
    );


    //Path follower request client
    path_request_client_ = this->create_client<mtrx3760_lrl_interfaces::srv::PathReq>
    (
        "/path_follower/path_request"
    );


    //Init path finder
    path_finder = PlanningCore(0.1);


    RCLCPP_INFO(this->get_logger(), "Path follower client created.");

}

// Destructor
DeliveryController::~DeliveryController() 
{
    RCLCPP_INFO(this->get_logger(), "Delivery Controller Node has been terminated.");
}




void DeliveryController::delivery_action_execute(const std::shared_ptr<GoalHandlePerformDelivery> goal_handle){
    //Get goal
    const auto goal = goal_handle->get_goal();


    //Perform delivery - stop map callback (to store most recent map), construct struct and link to path-finder
    PlanningCore::SlamMap100 slam;
    slam.occ100 = std::move(most_recent_map->data);

    slam.rows = most_recent_map->info.height;
    slam.cols = most_recent_map->info.width;
    slam.resolution_m = most_recent_map->info.resolution;
    slam.occ_thresh100 = 65;         // >= occupied
    slam.unknown_as_occupied = true; // -1 treated as occupied (safer)


    if (!path_finder.linkMap(slam)) {
        std::cerr << "Failed to link map\n";
        return;
    }


    // Obstacle inflation (meters).
    path_finder.inflate(0.2);

    
    struct Point {
        double x;
        double y;
    };

    //NEED TO SUB TO TF TO GET ROBOT POSE
    Point relative_pos = Point{curr_pose->x - most_recent_map->info.origin.position.x, 
                                curr_pose->y - most_recent_map->info.origin.position.x};

    
    path_finder.setStartCell(static_cast<int>(relative_pos.x/slam.resolution_m), 
                        static_cast<int>(relative_pos.y/slam.resolution_m));



    std::vector<PlanningCore::ItemTarget> items; 

    for (const auto pack : goal->package_requests){
        items.push_back(PlanningCore::ItemTarget{"ItemA", tag_dictionary_[pack.id].global_pos.x * 1000, 
                                                        tag_dictionary_[pack.id].global_pos.y * 1000, 'A',
                                                        pack.priority});
    }
    
    
    
    path_finder.setItems(items);

    //Plan route
    std::vector<GridMap::Cell> fullPath;
    std::vector<GroupReport> reports;
    if (!path_finder.plan(fullPath, reports)) {
        std::cerr << "Route infeasible\n";
        return;
    }

    //Extract commands from path-finder
    auto cmds = path_finder.buildCommands(/*fixedInitialHeading=*/std::nullopt);
    send_path_request(cmds);
}



// Callback function for marker detection from camera node (This is just so that the controller has access to package data)
void DeliveryController::package_detection_callback(const mtrx3760_lrl_interfaces::msg::MarkerDetection::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Package %d detected", msg->id);

    // Log in dictionary of packages (Dictionary to map priorities to packages)
    tag_dictionary_[msg->id];
    tag_dictionary_[msg->id].global_pos = Point{msg->global_position.x,
                                                msg->global_position.y};

    tag_dictionary_[msg->id].ID = msg->id;
    
}




void DeliveryController::send_path_request(std::vector<MotionCmd> cmds)
{
    // Wait for the service to be available
    while (!path_request_client_->wait_for_service(std::chrono::seconds(1))) 
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for path request service to be available...");
    }


    // Creating a request object for the PathReq service
    //auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    //mtrx3760_lrl_interfaces::srv::PathReq::Response
    auto request = std::make_shared<mtrx3760_lrl_interfaces::srv::PathReq::Request>();

    //REPLACE LATER!!!! CONVERT OLLIE PATH TO PATH FOLLOWER PACKET
    for (const auto cmd : cmds){
        if (cmd.type == CmdType::Translate){
            request->distances.push_back(cmd.value);

        } else if (cmd.type == CmdType::Rotate){
            request->angles.push_back(cmd.value * 90.0);

        }

    }

    RCLCPP_INFO(this->get_logger(), "Sending path request to Path Follower Node.");

    // Asynchronously call the service
    auto result_future = path_request_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Path request service call failed :(");
        path_request_client_->remove_pending_request(result_future);
        return;
    }

    return;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeliveryController>());
  rclcpp::shutdown();

  return 0;
}
