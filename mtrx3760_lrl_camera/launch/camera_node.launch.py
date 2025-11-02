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
    
    # Source the workspace setup
    source_setup = ExecuteProcess(
        cmd=['bash', '-c', 'source install/setup.bash && echo "Workspace sourced successfully"'],
        output='screen'
    )
    
    # Launch the camera node
    camera_node = Node(
        package='mtrx3760_lrl_camera',
        executable='camera',
        name='camera_node',
        output='screen',
        parameters=[]  # You can add parameters here if needed
    )
    
    return LaunchDescription([
        ros_domain_id_arg,
        set_ros_domain,
        camera_node
    ])
