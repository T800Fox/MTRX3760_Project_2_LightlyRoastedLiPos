#include "mtrx3760_lrl_package_delivery/pathFollower_node.hpp"


pathFollower::pathFollower()
: Node("path_follower_node"), actuator_client_wrapper_() {

    this->delayed_wrapper_init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        [this](){

            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

            //Init path-follow server
            follow_path_server = this->create_service<mtrx3760_lrl_interfaces::srv::PathReq>(
                    "follow_path", std::bind(&pathFollower::follow_path_callback, this, std::placeholders::_1, std::placeholders::_2));

            //Init actuator wrapper
            auto node_shared_ptr = std::static_pointer_cast<rclcpp::Node>(this->shared_from_this());
            actuator_client_wrapper_ = ActuatorClientWrapper(node_shared_ptr, "actuator");

            RCLCPP_INFO(this->get_logger(), "Path follower has been initialised");
            this->delayed_wrapper_init_timer_->cancel();
        }
    );

}

pathFollower::~pathFollower(){}


//Follow path server callback
void pathFollower::follow_path_callback(
    const std::shared_ptr<mtrx3760_lrl_interfaces::srv::PathReq::Request> request,
    std::shared_ptr<mtrx3760_lrl_interfaces::srv::PathReq::Response> response){

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
        //Update timer
        this->update_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                            [this](){update();});
}




void pathFollower::update(){
    //If stationary, pass next command
    RCLCPP_INFO(this->get_logger(), "ahhh");
    if (actuator_client_wrapper_.is_awaiting_result()){return;}

    RCLCPP_INFO(this->get_logger(), "Next cmd!!!");

    switch ((CMD_TYPE) (cmd_ind % 2)){
        case ROTATE:{
            ActuatorCmd ang_cmd{ang_cmd.MODE_ABS_ANGULAR, 3.1415926/180.0 * angles[cmd_ind/2]};
            //Publish angle to controller
            actuator_client_wrapper_.send_goal(ang_cmd, nullptr, nullptr);
            break;
        }

        case DRIVE:{
            ActuatorCmd lin_cmd{lin_cmd.MODE_ABS_LINEAR, distances[cmd_ind/2]};
            //Publish distance to controller
            actuator_client_wrapper_.send_goal(lin_cmd, nullptr, nullptr);
            break;
        }
    }

    //Increment command index and check if path has been complete
    if (cmd_ind ++ == distances.size()*2){
        RCLCPP_INFO(this->get_logger(), "Path completed");
        this->update_timer_->cancel();
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