from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mtrx3760_lrl_package_delivery',
            executable='pathFollower',
            name='wall_locator',
            output='screen'
        ),
        Node(
            package='mtrx3760_lrl_package_delivery',
            executable='deliveryController',
            name='delivery_controller',
            output='screen'
        ),

    ])
