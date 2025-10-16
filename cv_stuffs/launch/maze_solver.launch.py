from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mtrx3760_oogway_mazesolver',
            executable='wallLocator',
            name='wall_locator',
            output='screen'
        ),
        Node(
            package='mtrx3760_oogway_mazesolver',
            executable='mazeNavigator',
            name='maze_navigator',
            output='screen'
        ),
        Node(
            package='mtrx3760_oogway_mazesolver',
            executable='robotActuator',
            name='robot_actuator',
            output='screen'
        ),
        Node(
            package='mtrx3760_oogway_mazesolver',
            executable='goalChecker',
            name='goal_checker',
            output='screen'
        )
    ])
