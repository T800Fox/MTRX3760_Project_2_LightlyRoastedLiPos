#!/bin/bash

# Remote robot deployment script
echo "Deploying to TurtleBot3..."

# Set robot IP (change this to your robot's IP)
ROBOT_IP="10.70.139.15"
ROBOT_USER="ubuntu"
ROBOT_PASSWORD="turtlebot"

echo "Connecting to robot at $ROBOT_IP..."

# Copy your package to the robot
echo "Copying package to robot..."
scp -r . $ROBOT_USER@$ROBOT_IP:~/marker_tracking/

# SSH into robot and run setup
ssh $ROBOT_USER@$ROBOT_IP << EOF
    echo "On robot: Building package..."
    cd ~/marker_tracking
    colcon build --symlink-install
    
    echo "On robot: Starting hardware bringup..."
    # Start hardware bringup in background
    ros2 launch turtlebot3_bringup robot.launch.py &
    sleep 5
    
    echo "On robot: Starting camera..."
    # Start camera in background
    ros2 launch turtlebot3_bringup camera.launch.py &
    sleep 5
    
    echo "On robot: Starting your camera node..."
    # Start your camera node
    source install/setup.bash
    export ROS_DOMAIN_ID=15
    ros2 launch marker_tracking camera_node.launch.py
EOF
