from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera Node (now includes controller functionality)
        Node(
            package='mtrx3760_lrl_camera',
            executable='camera',
            name='camera_node',
            output='screen',
            parameters=[{
                'confidence_threshold': 0.7,  # Green confidence
                'max_observations_per_id': 10
            }]
        )
    ])

