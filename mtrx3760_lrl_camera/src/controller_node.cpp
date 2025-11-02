#include <controller_node.hpp>
#include <cmath>
using namespace mtrx3760_lrl_camera::msg;

ControllerNode::ControllerNode() : Node("odom_fusion_controller"), 
    robot_x(0.0), robot_y(0.0), robot_theta(0.0)
{
    // Declare parameters
    this->declare_parameter("confidence_threshold", 0.7);
    this->declare_parameter("max_observations_per_id", 10);
    
    confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
    max_observations_per_id_ = this->get_parameter("max_observations_per_id").as_int();
    
    // Initialize subscribers
    detection_sub_ = create_subscription<DetectedMarker>(
        "/detections/raw", 10,
        std::bind(&ControllerNode::detection_callback, this, std::placeholders::_1));
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ControllerNode::odom_callback, this, std::placeholders::_1));
    
    // Initialize publisher
    positions_pub_ = create_publisher<MarkerPositions>("/marker_positions", 10);
    
    RCLCPP_INFO(this->get_logger(), "Odom Fusion Controller started");
    RCLCPP_INFO(this->get_logger(), "Confidence threshold: %.2f", confidence_threshold_);
    RCLCPP_INFO(this->get_logger(), "Max observations per ID: %d", max_observations_per_id_);
}

ControllerNode::~ControllerNode()
{
    RCLCPP_INFO(this->get_logger(), "Odom Fusion Controller terminated");
}

void ControllerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    
    // Extract yaw from quaternion
    auto& q = msg->pose.pose.orientation;
    robot_theta = atan2(2.0 * (q.w * q.z + q.x * q.y), 
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void ControllerNode::detection_callback(const DetectedMarker::SharedPtr msg)
{
    // Filter by confidence (green = >0.7)
    if (msg->confidence < confidence_threshold_) {
        return;
    }
    
    // Transform from camera frame to global frame
    double cos_theta = cos(robot_theta);
    double sin_theta = sin(robot_theta);
    
    double camera_x_in_robot = msg->position.x;
    double camera_y_in_robot = msg->position.y;
    
    // Rotate to global frame
    double global_x = robot_x + camera_x_in_robot * cos_theta - camera_y_in_robot * sin_theta;
    double global_y = robot_y + camera_x_in_robot * sin_theta + camera_y_in_robot * cos_theta;
    double global_z = msg->position.z;
    
    // Add observation to storage
    add_observation(msg->id, global_x, global_y, global_z, msg->confidence);
    
    // Publish updated positions (event-driven)
    publish_averaged_positions();
}

void ControllerNode::add_observation(int32_t id, double x, double y, double z, double confidence)
{
    auto now = this->now();
    
    // Add new observation
    marker_storage_[id].push_back(MarkerObservation(x, y, z, confidence, now));
    
    // Evict old observations if limit exceeded
    evict_old_observations(id);
}

void ControllerNode::evict_old_observations(int32_t id)
{
    auto& observations = marker_storage_[id];
    
    // Keep only most recent N observations
    if (observations.size() > max_observations_per_id_) {
        observations.erase(observations.begin(), 
                          observations.begin() + 
                          (observations.size() - max_observations_per_id_));
    }
}

void ControllerNode::publish_averaged_positions()
{
    auto msg = MarkerPositions();
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";
    
    for (const auto& pair : marker_storage_) {
        int32_t id = pair.first;
        const auto& observations = pair.second;
        
        if (observations.empty()) continue;
        
        // Calculate weighted average
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0, sum_conf = 0.0;
        
        for (const auto& obs : observations) {
            sum_x += obs.x * obs.confidence;
            sum_y += obs.y * obs.confidence;
            sum_z += obs.z * obs.confidence;
            sum_conf += obs.confidence;
        }
        
        if (sum_conf > 0) {
            MarkerPosition pos;
            pos.id = id;
            pos.global_position.x = sum_x / sum_conf;
            pos.global_position.y = sum_y / sum_conf;
            pos.global_position.z = sum_z / sum_conf;
            pos.confidence = sum_conf / observations.size();
            pos.observation_count = observations.size();
            
            // Single line debug output - just show the updated average
            RCLCPP_INFO(this->get_logger(), 
                "ID=%d avg=(%.2f, %.2f, %.2f) n=%d",
                id, pos.global_position.x, pos.global_position.y, pos.global_position.z, 
                pos.observation_count);
            
            msg.markers.push_back(pos);
        }
    }
    
    positions_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}

