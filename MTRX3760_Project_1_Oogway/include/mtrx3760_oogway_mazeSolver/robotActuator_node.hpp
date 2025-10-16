#ifndef MTRX3760_OOGWAY_MAZESOLVER_ROBOTACTUATION_NODE_HPP_
#define MTRX3760_OOGWAY_MAZESOLVER_ROBOTACTUATION_NODE_HPP_

#include <memory>
#include <cmath>
#include <algorithm>


#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/bool.hpp>

#include "mtrx3760_oogway_mazesolver/msg/angular_cmd.hpp"
#include "mtrx3760_oogway_mazesolver/msg/linear_cmd.hpp"
#include "mtrx3760_oogway_mazesolver/msg/pose.hpp"
#include "mtrx3760_oogway_mazeSolver/utils.hpp"


const double ANGLE_ACCURACY_THRESH = 0.005f;
const double DIST_ACCURACY_THRESH = 0.02f;

const double MAX_ANG_VEL = 0.8f;
const double MAX_LIN_VEL = 0.2f;






class robotActuator : public rclcpp::Node
{
    public:
        robotActuator();
        ~robotActuator();
    private:
        // ROS topic publishers
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_abs_rotating_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_abs_moving_pub_;
        rclcpp::Publisher<mtrx3760_oogway_mazesolver::msg::Pose>::SharedPtr curr_pose_pub_;

        // ROS topic publishers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
        rclcpp::Subscription<mtrx3760_oogway_mazesolver::msg::AngularCmd>::SharedPtr angular_cmd_sub_;
        rclcpp::Subscription<mtrx3760_oogway_mazesolver::msg::LinearCmd>::SharedPtr linear_cmd_sub_;

        // Ros services

        // Timers
        rclcpp::TimerBase::SharedPtr angle_feedback_timer_;
        rclcpp::TimerBase::SharedPtr linear_feedback_timer_;

        // Callbacks
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void angle_feedback_callback();
        void angular_cmd_callback(const mtrx3760_oogway_mazesolver::msg::AngularCmd::SharedPtr msg);

        void linear_feedback_callback();
        void linear_cmd_callback(const mtrx3760_oogway_mazesolver::msg::LinearCmd::SharedPtr msg);

        Pose2D curr_pose; //Robots current pose in 2d plane (abstracted from odom data)
        double target_angle; //Target angle for feedback loop to achieve
        double target_dist; //Target dist for feedback loop to achieve
        Pose2D starting_pose; //Starting pose is recorded for smoothening motion (odometry accuracy loss from slipping) ???????!!!!!!!!!!!!!!!!!!

        double prev_ang_vel; //Previous ang-vel so as to not override previous
        double prev_lin_vel; ////Previous linear-vel so as to not override previous

        //Record of previous rotation/motion-state to avoid excessive publishing
        bool last_is_abs_rotating; 
        bool last_is_abs_moving; 

};

#endif