from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

PKG = 'parameters_manager_ex'

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory(PKG),
        'config',
        'example_node.yaml'
    )

    return LaunchDescription([
        Node(
            package=PKG,
            executable='example_node.py',
            parameters=[param_file]
        )
    ])
