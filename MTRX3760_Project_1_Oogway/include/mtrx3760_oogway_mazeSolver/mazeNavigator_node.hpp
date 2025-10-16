#ifndef MTRX3760_OOGWAY_MAZESOLVER_MAZENAVIGATOR_NODE_HPP_
#define MTRX3760_OOGWAY_MAZESOLVER_MAZENAVIGATOR_NODE_HPP_

#include <memory>
#include <algorithm> 
#include <cmath>
#include <deque>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

//Msgs
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "mtrx3760_oogway_mazesolver/msg/wall_dist.hpp"
#include "mtrx3760_oogway_mazesolver/msg/pose.hpp"
#include "std_msgs/msg/float64.hpp"

#include "mtrx3760_oogway_mazesolver/msg/angular_cmd.hpp"
#include "mtrx3760_oogway_mazesolver/msg/linear_cmd.hpp"
#include "mtrx3760_oogway_mazeSolver/utils.hpp"


const double POSE_EQUAL_THRESH = 0.15;

//Services


enum DIRECTION {
    FORWARD,
    LEFT,
    BACKWARD,
    RIGHT,
};

enum STATE {
    CONTACT_WALL,
    AWAIT_LOGIC,
    ROT_RIGHT_CORNER,
};





const double LIDAR_DT = 1.0/5; // LIDAR update-rate is 5HZ (from .sdf file)
const double CORNER_OFFSET = 0.15; //Just below threshold distance to ensure wall is registered
const double STABLE_VEL = 1.0; //Linear velocity at which odom is stable (no slip)


class mazeNavigator : public rclcpp::Node
{
    public:
        mazeNavigator();
        ~mazeNavigator();
    private:
        // ROS topic publishers
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<mtrx3760_oogway_mazesolver::msg::AngularCmd>::SharedPtr angular_cmd_pub_;
        rclcpp::Publisher<mtrx3760_oogway_mazesolver::msg::LinearCmd>::SharedPtr linear_cmd_pub_;

        // ROS topic subscribers
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr at_goal_sub_;
        rclcpp::Subscription<mtrx3760_oogway_mazesolver::msg::WallDist>::SharedPtr wall_dist_sub_;
        rclcpp::Subscription<mtrx3760_oogway_mazesolver::msg::Pose>::SharedPtr curr_pose_sub_;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_abs_rotating_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_abs_moving_sub_;

        

        // Subscriber callbacks
        void at_goal_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void wall_dist_callback(const mtrx3760_oogway_mazesolver::msg::WallDist::SharedPtr msg);

        // Service clients

        // Service callers

        // Member variables
        bool is_abs_rotating;
        bool is_abs_moving;
        bool prev_at_init_pose;
        bool awaiting_loop_jump;
        double curr_angle;
        std::array<bool, 4> prev_wall_pres; //Presence of wall in each 90deg direction (at previous reading)

        Point prev_seg_end_point;
        std::vector<LineSeg> traversed_segs; //Array of line-segments that have been travesered already (for identifying foreign loops)
        double prev_right_dist; //Distance to right at previous update (for determining wall_postiion)
        
        uint64_t moving_avr_window;
        std::deque<double> moving_avr_buf; //Buffer of distances for angular refinement (deqeue allows efficient itteration and push/pop)
        double old_avr; //Moving average at previous update for calculating roc

        STATE state; //High-level navigation state
        int move_index;

        Pose2D curr_pose;
        Pose2D loop_init_pose;

        // Member functions
        void rotate_offset(double angle);
        void set_ang_vel(double vel);
        void move_distance(double distance);
        void set_vel(double vel);
        void refine_angle(double new_dist);





};

#endif