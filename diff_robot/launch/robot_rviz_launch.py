import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # File paths
    urdf_file_path = '/home/svr/ROBOMUSE_WS/src/diff_robot/urdf/robomuse.urdf'
    rviz_config_file_path = '/home/svr/ROBOMUSE_WS/src/diff_robot/urdf/rviz.rviz'

    # Load robot description from URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            name='model',
            default_value=urdf_file_path,
            description='Absolute path to robot URDF file'),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=rviz_config_file_path,
            description='Absolute path to RViz config file'),

        # Publish the robot state (robot_description topic)
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_desc}],
             remappings=[("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel")]),

        # Launch RViz
        Node(package='rviz2', executable='rviz2',
             name='rviz2',
             output='screen',
             arguments=['-d', LaunchConfiguration('rvizconfig')]),
    ])
