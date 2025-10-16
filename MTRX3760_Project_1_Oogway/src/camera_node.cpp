#include "mtrx3760_oogway_mazeSolver/camera_node.hpp"


camera::camera(): Node("oogway_camera_node"){
    //Init subscriber
    camera_sub_ = create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, 
        std::bind(&camera::camera_callback, this, std::placeholders::_1));
}

camera::~camera()
{
  RCLCPP_INFO(this->get_logger(), "Oogway Goal Checker simulation node has been terminated");
}


void camera::camera_callback(const sensor_msgs::msg::Image::SharedPtr image){
    cv_bridge::CvImagePtr cvPointer;
    try {
        cvPointer = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    
    } catch (const cv_bridge::Exception & e){
        return;
    }

    cv::Mat frame = cvPointer->image;
    cv::imshow("Frame", frame);
    cv::waitKey(1);
}



//Spin
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camera>());
  rclcpp::shutdown();

  return 0;
}