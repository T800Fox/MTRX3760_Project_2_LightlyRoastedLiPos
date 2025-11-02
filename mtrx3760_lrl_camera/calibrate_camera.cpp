#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <iostream>
#include <vector>

class CameraCalibrator : public rclcpp::Node {
public:
    CameraCalibrator() : Node("camera_calibrator") {
        // ArUco calibration parameters
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        tag_size = 0.09; // 9cm tags
        
        // Subscribe to the same camera topic as your camera node
        camera_sub_ = create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, 
            std::bind(&CameraCalibrator::camera_callback, this, std::placeholders::_1));
        
        std::cout << "Camera calibration using ArUco tags" << std::endl;
        std::cout << "Place ArUco tag at different distances and angles" << std::endl;
        std::cout << "Press 'c' to capture calibration data, 'q' to quit" << std::endl;
        std::cout << "Waiting for camera images..." << std::endl;
    }
    
    void camera_callback(const sensor_msgs::msg::Image::SharedPtr image) {
        cv_bridge::CvImagePtr cvPointer;
        try {
            cvPointer = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception & e) {
            return;
        }
        
        cv::Mat frame = cvPointer->image;
        
        // Detect ArUco markers
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dictionary, corners, ids);
        
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            
            // For each detected marker
            for (size_t i = 0; i < ids.size(); i++) {
                // Define 3D points of the marker (assuming it's flat on a surface)
                std::vector<cv::Point3f> marker_points;
                marker_points.push_back(cv::Point3f(-tag_size/2, -tag_size/2, 0));
                marker_points.push_back(cv::Point3f(tag_size/2, -tag_size/2, 0));
                marker_points.push_back(cv::Point3f(tag_size/2, tag_size/2, 0));
                marker_points.push_back(cv::Point3f(-tag_size/2, tag_size/2, 0));
                
                // Add to calibration data
                for (const auto& point : marker_points) {
                    object_points.push_back(point);
                }
                for (const auto& corner : corners[i]) {
                    image_points.push_back(corner);
                }
            }
        }
        
        cv::imshow("Calibration", frame);
        
        char key = cv::waitKey(1);
        if (key == 'q') {
            perform_calibration();
            rclcpp::shutdown();
            return;
        }
        if (key == 'c') {
            std::cout << "Captured " << object_points.size() << " calibration points" << std::endl;
        }
    }
    
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    double tag_size;
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    
    void perform_calibration() {
        // Perform camera calibration
        if (object_points.size() > 20) { // Need at least 20 points
            cv::Mat camera_matrix, dist_coeffs;
            std::vector<cv::Mat> rvecs, tvecs;
            
            double rms = cv::calibrateCamera(
                std::vector<std::vector<cv::Point3f>>(1, object_points),
                std::vector<std::vector<cv::Point2f>>(1, image_points),
                cv::Size(640, 480),
                camera_matrix,
                dist_coeffs,
                rvecs,
                tvecs
            );
            
            std::cout << "Calibration RMS error: " << rms << std::endl;
            std::cout << "Camera matrix:" << std::endl << camera_matrix << std::endl;
            std::cout << "Distortion coefficients:" << std::endl << dist_coeffs << std::endl;
            
            // Save calibration data
            cv::FileStorage fs("camera_calibration.yml", cv::FileStorage::WRITE);
            fs << "camera_matrix" << camera_matrix;
            fs << "distortion_coefficients" << dist_coeffs;
            fs.release();
            
            std::cout << "Calibration data saved to camera_calibration.yml" << std::endl;
        } else {
            std::cout << "Not enough calibration points. Need at least 20." << std::endl;
        }
        
        cv::destroyAllWindows();
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraCalibrator>());
    rclcpp::shutdown();
    return 0;
}
