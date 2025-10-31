# ros2 run nav2_map_server map_saver_cli -f <map_name>
# ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=my_map.yaml

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    amcl_params_file = os.path.join(get_package_share_directory("mtrx3760_lrl_warehousebot"),
                                    'config', 
                                    'amcl_params.yaml') 
    return LaunchDescription([
        Node(
            package='nav2_amcl',  
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params_file]
        )
    ])