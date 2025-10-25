#ifndef MRTX3760_OOGWAY_MAZESOLVER__ACTUATOR_NODE_HPP_
#define MRTX3760_OOGWAY_MAZESOLVER__ACTUATOR_NODE_HPP_

#include <memory>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include "mtrx3760_oogway_mazesolver/msg/angular_cmd.hpp"
#include "mtrx3760_oogway_mazesolver/msg/linear_cmd.hpp"
#include "mtrx3760_oogway_mazesolver/msg/pose.hpp"
#include "mtrx3760_oogway_mazesolver/msg/utils.hpp"


const double ANGLE_ACCURACY_THRESH = 0.005f;
const double LINEAR_ACCURACY_THRESH = 0.02f;

const double MAX_ANG_VEL = 0.8f;
const double MAX_LIN_VEL = 0.2f;




class robotActuator : public rclcpp::Node
{
public:
    robotActuator();
    ~robotActuator();

private:
    // Publishers   
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_abs_rotating_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_abs_moving_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr curr_pose_pub;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
    rclcpp::Subscription<mtrx3760_oogway_mazesolver::msg::AngularCmd>::SharedPtr angular_cmd_sub;
    rclcpp::Subscription<mtrx3760_oogway_mazesolver::msg::LinearCmd>::SharedPtr linear_cmd_sub;
    rclcpp::Subscription<mtrx3760_oogway_mazesolver::msg::NavCommand>::SharedPtr nav_cmd_sub;
   
    //Ros services

    //Timers
    rclcpp::TimerBase::SharedPtr angle_feedback_timer_;
    rclcpp::TimerBase::SharedPtr linear_feedback_timer_;

    //Callback functions
    void odom_callback(const nav_msgs::msg::Odometry>::SharedPtr msg);
    void angular_cmd_callback(const mtrx3760_oogway_mazesolver::msg::AngularCmd>::SharedPtr msg);
    void linear_cmd_callback(const mtrx3760_oogway_mazesolver::msg::LinearCmd>::SharedPtr msg);
    void nav_cmd_callback(const mtrx3760_oogway_mazesolver::msg::NavCommand::SharedPtr msg);

    void angle_feedback_callback();
    void linear_feedback_callback();

    //Internal state variables
    Pose2D starting_pose;
    Pose2D curr_pose;

    double target_angle; //Target angle for feedback loop to achieve
    double target_dist; //Target distance for feedback loop to achieve
    double prev_ang_val; //previous ang-vel so as to not override previous
    double prev_lin_val; //previous lin-vel so as to not override previous

    //record of previous rotation/motion state to prevent redundant publishing
    bool last_is_abs_rotating;  
    bool last_is_abs_moving;




};

#endif // MRTX3760_OOGWAY_MAZESOLVER__ACTUATOR_NODE_HPP_