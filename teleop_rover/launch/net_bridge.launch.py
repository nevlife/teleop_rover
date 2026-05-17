import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('teleop_rover')
    network_file = os.path.join(pkg, 'config', 'network.yaml')
    params_file = os.path.join(pkg, 'config', 'net_bridge_params.yaml')

    net_bridge_node = Node(
        package='teleop_rover',
        executable='net_bridge.py',
        name='net_bridge',
        output='screen',
        parameters=[
            network_file,
            params_file,
        ],
    )

    return LaunchDescription([
        net_bridge_node,
    ])
