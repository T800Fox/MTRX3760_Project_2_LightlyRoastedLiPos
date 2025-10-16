#include "camera_node.hpp"


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

    //Convert to hsv for easier colour recognition
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    //Colour range for red (accounting for lighting conditions)
    cv::Scalar lower_red(0,100,100);
    cv::Scalar upper_red(10,255,255);

    cv::Mat mask_red;
    cv::inRange(hsv, lower_red, upper_red, mask_red);


    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(mask_red, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (auto &contour : contours) {
        cv::Moments m = cv::moments(contour);
        if (m.m00 > 100) {  // filter small noise
            int cx = int(m.m10 / m.m00);
            int cy = int(m.m01 / m.m00);
            cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(0,0,255), -1);
        }
    }


    
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