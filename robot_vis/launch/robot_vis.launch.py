from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_vis',
            executable='ros2Interface',
            name='ros2_interface_node',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_vis', 'my_app'],
            output='screen'
        )

    ])
