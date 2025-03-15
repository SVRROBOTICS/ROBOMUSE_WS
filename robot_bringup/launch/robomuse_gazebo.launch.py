import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # File paths
    urdf_file_path = '/home/svr/ROBOMUSE_WS/src/diff_robot/urdf/robomuse.urdf'
    rviz_config_file_path = '/home/svr/ROBOMUSE_WS/src/diff_robot/urdf/robomuse_rviz.rviz'
    world_file_path = '/home/svr/ROBOMUSE_WS/src/diff_robot/world/office_small.world'

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('/opt/ros/humble/share/gazebo_ros/launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.025'
    spawn_yaw_val = '0.0'

    # Load URDF content
    with open(urdf_file_path, 'r') as urdf_f:
        urdf_content = urdf_f.read()
    
    params = {'robot_description': urdf_content}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params, {'publish_frequency': 20.0, 'use_sim_time': True}]
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robomuse',
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', spawn_z_val,
            '-Y', spawn_yaw_val
        ],
        output='screen'
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file_path],
        parameters=[{'use_sim_time': True}]
    )

    # Static transform from map to odom
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        joint_state_publisher_node,
        rviz,
        static_transform_publisher
    ])
