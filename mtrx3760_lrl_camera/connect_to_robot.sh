#!/bin/bash

# Quick robot connection script
echo "Connecting to TurtleBot3..."

# Set robot IP (change this to your robot's IP)
ROBOT_IP="10.70.139.15"
ROBOT_USER="ubuntu"

echo "SSH into robot at $ROBOT_IP..."
echo "Password: turtlebot"

# SSH into robot
ssh $ROBOT_USER@$ROBOT_IP
