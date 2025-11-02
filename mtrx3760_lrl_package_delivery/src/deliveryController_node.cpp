#include "mtrx37060_LRL_warehousebot/DeliveryController_node.hpp"


DeliveryController::DeliveryController()
: Node("Delivery Controller node") // Constructor
{
    
    using namespace std::placeholders;

    //---Warenouse Inspection Server Setup---
    auto handle_goal = [this] (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PerformDelivery::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request -> UUID = %s", rclcpp_action::to_string(uuid).c_str());

        //Confirm that inspection can be performed (inspection not already in progress)
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT;
    }

    auto handle_cancel = [this] (const std::shared_ptr<GoalHandlePerformDelivery> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [this] (const std::shared_ptr<GoalHandlePerformDelivery> goal_handle)
    {
        //Execute
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&DeliveryController::delivery_action_execute, this, _1), goal_handle}.detach();

    };


    this->delivery_action_server_ = rclcpp_action::create_server<PerformDelivery>(
        this,
        "inspect_warehouse",
        handle_goal,
        handle_cancel,
        handle_accepted
    );


    //--------------------ROS SUBSCRIBERS-------------------//
    // Subscriber for package data received from camera node
    package_detection_subscriber_= this->create_subscription<mtrx3760_lrl_warehousebot::msg::PackageDetection>
    (
        "/camera/markerposition", 
        10, 
        std::bind
        (
            &DeliveryController::package_detection_callback, 
            this, 
            std::placeholders::_1
        )
    );


    //Occupancy grid subscriber
    occupancy_grid_subscriber_ = this->create_subscription<mtrx3760_lrl_warehousebot::msg::OccupancyGrid>
    (
        "/map",
        10,
        [this](const mtrx3760_lrl_warehousebot::msg::OccupancyGrid::SharedPtr msg){
            most_recent_map = msg;
        }
    );


    //Path follower request client
    path_request_client_ = this->create_client<mtrx3760_lrl_warehousebot::srv::PathReq>
    (
        "/path_follower/path_request"
    );


    //Init path finder
    path_finder(0.1);


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
    slam.rows = most_recent_map->info->height;
    slam.cols = most_recent_map->info->width;
    slam.resolution_m = most_recent_map->info->resolution;
    slam.occ_thresh100 = 65;         // >= occupied
    slam.unknown_as_occupied = true; // -1 treated as occupied (safer)

    if (!planner.linkMap(slam)) {
        std::cerr << "Failed to link map\n";
        return 1;
    }

    // Obstacle inflation (meters).
    planner.inflate(0.2);

    planner.setStartCell(/*Find current cell from odom*/);

    //del_cell = goal->delivery_cell
    //planer.setDeliveryCell(del_cell.x, del_cell.y) -----------------


    //Extract from goal
    std::vector<PlanningCore::ItemTarget> items = {
        {300, 1800, 2},
        {400,  900, 2},
        {300,  100, 1}
    };
    planner.setItems(items);

    //Plan route
    std::vector<GridMap::Cell> fullPath;
    std::vector<GroupReport> reports;
    if (!planner.plan(fullPath, reports)) {
        std::cerr << "Route infeasible\n";
        return 2;
    }

    //Extract commands from path-finder
    auto cmds = planner.buildCommands(/*fixedInitialHeading=*/std::nullopt);

    send_path_request(cmds);
}



// Callback function for marker detection from camera node (This is just so that the controller has access to package data)
void DeliveryController::package_detection_callback(const mtrx3760_lrl_warehousebot::msg::PackageDetection::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Package %d detected", msg->id);

    // Log in dictionary of packages (Dictionary to map priorities to packages)
    tag_dictionary_[msg->id] = msg;
}




void DeliveryController::send_path_request(std::vector<MotionCmd> cmds)
{
    // Wait for the service to be available
    while (!path_request_client_->wait_for_service(std::chrono::seconds(1))) 
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for path request service to be available...");
    }


    // Creating a request object for the PathReq service
    auto request = std::make_shared<mtrx3760_lrl_warehousebot::srv::PathReq::Request>();

    //REPLACE LATER!!!! CONVERT OLLIE PATH TO PATH FOLLOWER PACKET
    for (const auto cmd & : cmds){
        if (cmd.type == Translate){
            request->distances.push_back(cmd.value);

        } else if (cmd.type == Rotate){
            request->angles.push_back(cmd.value * 90.0);

        }

    }

    RCLCPP_INFO(this->get_logger(), "Sending path request to Path Follower Node.");

    // Asynchronously call the service
    auto result_future = path_request_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Path request service call failed :(");
        client->remove_pending_request(result_future);
        return 1;
    }
    

    return 0;
}
