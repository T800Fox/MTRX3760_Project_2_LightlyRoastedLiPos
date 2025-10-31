#include "mtrx3760_lrl_package_delivery/pathFollower_node.hpp"


pathFollower::pathFollower()
: Node("oogway_pathFollower_node"){

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    //WILL BE CHANGED WHEN ACTUATOR CHANGES TO SERVICE!!!
    is_abs_rotating_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "is_abs_rotating", 
        qos, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            is_abs_rotating = msg->data;
            update();
        }
    );

    is_abs_moving_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "is_abs_moving", 
        qos, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            is_abs_moving = msg->data;
            update();
        }
    );

    //Init actuator command publishers
    angular_cmd_pub_ = this->create_publisher<mtrx3760_oogway_mazesolver::msg::AngularCmd>("angular_cmd", qos); 
    linear_cmd_pub_ = this->create_publisher<mtrx3760_oogway_mazesolver::msg::LinearCmd>("linear_cmd", qos); 
    

    //Init path-follow server
    follow_path_server = this->create_service<mtrx3760_oogway_mazesolver::srv::PathReq>(
            "follow_path", std::bind(&pathFollower::follow_path_callback, this, std::placeholders::_1, std::placeholders::_2));


    is_abs_moving = is_abs_rotating = false;
    curr_angle = 0.0;

      RCLCPP_INFO(this->get_logger(), "Path follower has been initialised");

        
}

pathFollower::~pathFollower(){}


//Follow path server callback
void pathFollower::follow_path_callback(
    const std::shared_ptr<mtrx3760_oogway_mazesolver::srv::PathReq::Request> request,
    std::shared_ptr<mtrx3760_oogway_mazesolver::srv::PathReq::Response> response){

        //Storing references to shared_ptr service-request will guarentee lifetime 
        distances = request->distances;
        angles = request->angles;

        //Validate request
        response->success = (distances.size() == angles.size());
        if (!response->success) {
            return;
        }

        cmd_ind = 0; //Start on first cmd

        update(); //Call first command
}





void pathFollower::update(){
    //If stationary, pass next command
    if (!(is_abs_moving || is_abs_rotating)){

        switch ((CMD_TYPE) (cmd_ind % 2)){
            case ROTATE:{
                mtrx3760_oogway_mazesolver::msg::AngularCmd cmd;
                double angle_offset = angles[cmd_ind/2]; //target angle

                cmd.target_angle = std::fmod((curr_angle + angle_offset * (M_PI/180)), M_PI*2) ; //Wrap angle to valid range
                cmd.mode = cmd.MODE_ABSOLUTE;

                angular_cmd_pub_->publish(cmd); //Publish angle to controller

                curr_angle = cmd.target_angle; // Update current angle

                break;
            }

            case DRIVE:{
                mtrx3760_oogway_mazesolver::msg::LinearCmd cmd;
                cmd.target_distance = std::max(0.0, distances[cmd_ind/2]); //Clamp distance to positive value
                cmd.mode = cmd.MODE_ABSOLUTE;

                linear_cmd_pub_->publish(cmd); //Publish distance to controller
                
                break;
            }

        }

        //Increment command index and check if path has been complete
        if (cmd_ind ++ == distances.size()*2){
            RCLCPP_INFO(this->get_logger(), "Path completed");
            cmd_ind = 0;
        } 

        
    }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pathFollower>());
  rclcpp::shutdown();

  return 0;
}