from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera Node
        Node(
            package='mtrx3760_lrl_camera',
            executable='camera',
            name='camera_node',
            output='screen'
        ),
        
        # Controller Node (odom fusion)
        Node(
            package='mtrx3760_lrl_camera',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[{
                'confidence_threshold': 0.7,  # Green confidence
                'max_observations_per_id': 10
            }]
        )
    ])

