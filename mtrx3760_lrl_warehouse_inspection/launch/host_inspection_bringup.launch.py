from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mtrx3760_lrl_camera'),
                    'launch',
                    'marker_tracking.launch.py'
                ])
            ]),
        ),
        Node(
            package='mtrx3760_lrl_actuator',
            executable='actuator_action_server',
            name='actuator_action_server',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mtrx3760_lrl_warehouse_inspection'),
                    'launch',
                    'maze_solver_launch.py'
                ])
            ]),
        )
    ])