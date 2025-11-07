from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_bringup', 'robot.launch.py'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'mtrx3760_lrl_actuator', 'actuator_action_server'],
            output='screen'
        ),

    ])
