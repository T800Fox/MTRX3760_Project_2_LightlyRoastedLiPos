#!/bin/bash

# Build and run script for marker_tracking package
echo "Building marker_tracking package..."

# Build the package
colcon build --symlink-install --event-handlers console_direct+

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "Build successful! Starting camera node..."
    
    # Source the workspace
    source install/setup.bash
    
    # Set ROS domain ID
    export ROS_DOMAIN_ID=15
    
    # Launch the camera node
    ros2 launch marker_tracking camera_node.launch.py
else
    echo "Build failed! Please check the errors above."
    exit 1
fi
