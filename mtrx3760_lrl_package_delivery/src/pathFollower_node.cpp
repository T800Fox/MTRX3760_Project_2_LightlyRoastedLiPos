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
    if (wrapper->is_in_motion()){return;}

    switch ((CMD_TYPE) (cmd_ind % 2)){
        case ROTATE:{
            ActuatorCmd ang_cmd{ActuatorCmd.MODE_ABS_ANGULAR, 3.1415926/180.0 * angles[cmd_ind/2]};
            //Publish angle to controller
            send_goal(ang_cmd);
            break;
        }

        case DRIVE:{
            ActuatorCmd lin_cmd{ActuatorCmd.MODE_ABS_LINEAR, distances[cmd_ind/2]};
            //Publish distance to controller
            send_goal(lin_cmd);
            break;
        }

    }

    //Increment command index and check if path has been complete
    if (cmd_ind ++ == distances.size()*2){
        RCLCPP_INFO(this->get_logger(), "Path completed");
        cmd_ind = 0;
    } 

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pathFollower>());
  rclcpp::shutdown();

  return 0;
}