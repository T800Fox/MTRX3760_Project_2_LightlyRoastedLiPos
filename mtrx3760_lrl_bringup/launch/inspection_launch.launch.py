from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_bringup', 'camera.launch.py'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'mtrx3760_lrl_camera', 'marker_tracking.launch.py'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'mtrx3760_lrl_warehouse_inspection', 'online_async_launch.py'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'mtrx3760_lrl_acmtrx3760_lrl_warehouse_inspectiont', 'warehouse_inspection.launch.py'],
            output='screen'
        )
    ])
