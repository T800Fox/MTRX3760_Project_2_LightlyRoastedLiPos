from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare launch arguments
    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='15',
        description='ROS 2 domain ID'
    )
    
    # Set environment variables
    set_ros_domain = SetEnvironmentVariable(
        'ROS_DOMAIN_ID',
        LaunchConfiguration('ros_domain_id')
    )
    
    # Launch the camera node
    camera_node = Node(
        package='marker_tracking',
        executable='camera',
        name='camera_node',
        output='screen',
        parameters=[]  # You can add parameters here if needed
    )
    
    # Launch TagDatabase node (when you create it)
    # tag_database_node = Node(
    #     package='marker_tracking',
    #     executable='tag_database',
    #     name='tag_database_node',
    #     output='screen'
    # )
    
    return LaunchDescription([
        ros_domain_id_arg,
        set_ros_domain,
        camera_node,
        # tag_database_node,  # Uncomment when ready
    ])
