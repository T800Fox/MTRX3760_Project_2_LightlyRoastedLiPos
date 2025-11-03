from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mtrx3760_lrl_warehouse_inspection',
            executable='wallLocator',
            name='wall_locator',
            output='screen'
        ),
        Node(
            package='mtrx3760_lrl_warehouse_inspection',
            executable='inspectWarehouseController',
            name='inspect_warehouse_controller',
            output='screen'
        ),

    ])
