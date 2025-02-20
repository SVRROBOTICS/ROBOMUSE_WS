import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'robot_bringup'  # Replace with your actual package name
    config_file_path = os.path.join(get_package_share_directory(package_name), 'config', 'map_server_params.yaml')
    lifecycle_nodes = ['map_server',
                       ]

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[config_file_path]
        ),
        Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}]),
    ])
