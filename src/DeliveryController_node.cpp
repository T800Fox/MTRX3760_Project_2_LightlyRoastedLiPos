#include "mtrx37060_LRL_warehousebot/DeliveryController_node.hpp"


DeliveryController::DeliveryController()
: Node("Delivery Controller node") // Constructor
{
    
    RCLCPP_INFO(this->get_logger(), "Delivery Controller Node has been started.");


    //--------------------ROS SUBSCRIBERS-------------------//

    // Subscriber for package data received from camera node
    package_subscriber_= this->create_subscription<mtrx3760_lrl_warehousebot::msg::PackageDetection>
    (
        "/camera/markerposition", 
        10, 
        std::bind
        (
            &DeliveryController::markerCallback, 
            this, 
            std::placeholders::_1
        )
    );


    // Subscriber for user interface required packages E.g. package A, B, C, priority levels etc. (Might change to service later)
    interface_subscriber_ = this->create_subscription<mtrx3760_lrl_warehousebot::msg::UICommand>
    (
        "/ui/commands", 
        10, 
        std::bind
        (
            &DeliveryController::interfaceCallback, 
            this, 
            std::placeholders::_1
        )
    );

    // Subscriber for occupancy grid map data from SLAM node to pass to path finder
    occupancygrid_subscriber_ = this->create_subscription<mtrx3760_lrl_warehousebot::msg::OccupancyGrid>
    (
        "/map",
        10,
        std::bind
        (
            &DeliveryController::occupancyGridCallback,
            this,
            std::placeholders::_1
        )
    );



     /*--------------------CONTROLLER SHOULD HAVE NO ROS PUBLISHERS OR SUBSCRIBERS
    ----------------------FOR PATH FINDER AS PATH FINDER IS NOT A NODE-------------------*/

    //DONT NEED THIS AS WE ARE NOT SENDING PROCESSED DATA ANYWHERE//
    // Publisher for processed package data to send to UI node
   /*package_publisher_ = this->create_publisher<mtrx3760_lrl_warehousebot::msg::PackageDetection>
    (
        "/camera/markerdictionary",
        10,
        std::bind
        (
            &DeliveryController::markerCallback, 
            this, 
            std::placeholders::_1
        )
    ); */



    // IMPLEMENT PATH REQUEST INFO LOGIC HERE TO SEND TO PATH FOLLOWER NODE //
    // SO THIS DELIVERY CONTROLLER NODE WILL BE THE CLIENT, AND PATH FOLLOWER WILL BE SERVER //
    // PATH FOLLOWER SERVES THIS CONTROLLER NODE BY SENDING A SUCCESS RESPONSE //

    path_request_client_ = this->create_client<mtrx3760_lrl_warehousebot::srv::PathReq>
    (
        "/path_follower/path_request"

    );

    RCLCPP_INFO(this->get_logger(), "Path follower client created.");

}

// Destructor
DeliveryController::~DeliveryController() 
{
    RCLCPP_INFO(this->get_logger(), "Delivery Controller Node has been terminated.");
}

// Callback function for marker detection from camera node (This is just so that the controller has access to package data)
void DeliveryController::markerCallback(const mtrx3760_lrl_warehousebot::msg::PackageDetection::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Package %d detected", msg->id);

    // Implement Dictionary of packages (Dictionary to map priorities to packages)
    tag_dictionary_[msg->id] = msg;
    RCLCPP_INFO(this->get_logger(), "Total unique packages in dictionary: %zu", tag_dictionary_.size());


    // Example to send path request to Path Follower Node
    std::vector<double> angles = {0.0, 1.57, 3.14};  // Example angles in radians
    std::vector<double> distances = {1.0, 0.5, 2.0}; // Example distances in meters
    
    sendPathRequest(angles, distances);
}

// Callback function for user interface commands (Package list, priorities)
void DeliveryController::interfaceCallback(const mtrx3760_lrl_warehousebot::msg::UICommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "UI Package Delivery List Request received: %s, priorities: %d", msg->package_list.c_str(), msg->package_priority);

    // Logic to give this data to path finder logic to plan path accordingly

}

// CHANGE THIS TO A NORMAL METHOD BECAUSE U ONLY WANNA PASS THE WHOLE FINISHED MAP AT THE END AND ONCE
void DeliveryController::occupancyGridCallback(const mtrx3760_lrl_warehousebot::msg::OccupancyGrid::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Occupancy Grid Map data received.");

    // Logic to pass this map data to path finder logic

}

void DeliveryController::sendPathRequest(const std::vector<double>& angles, const std::vector<double>& distances)
{
    // Wait for the service to be available
    while (!path_request_client_->wait_for_service(std::chrono::seconds(1))) 
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for path request service to be available...");
    }

    // Creating a request object for the PathReq service
    auto request = std::make_shared<mtrx3760_lrl_warehousebot::srv::PathReq::Request>();
    request->angles = angles;
    request->distances = distances;

    RCLCPP_INFO(this->get_logger(), "Sending path request to Path Follower Node.");

    // Asynchronously call the service
    auto result = path_request_client_->async_send_request(
        request,
        std::bind(&DeliveryController::pathRequestResponseCallback, this, std::placeholders::_1)
    );
}

void DeliveryController::pathRequestResponseCallback(rclcpp::Client<mtrx3760_lrl_warehousebot::srv::PathReq>::SharedPtr msg)
{
    // Logic to handle the response from the Path Follower Node

    if (msg->success)
    {
        RCLCPP_INFO(this->get_logger(), "Path Follower Node successfully completed path routine");

    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Path Follower Node failed to complete path routine");
        // Could add logic here to resend path request or handle failure
    }
}