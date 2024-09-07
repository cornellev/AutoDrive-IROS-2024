from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('autodrive_iros_2024'),  # Replace with your package name
        'config', 
        'robot_localization.yaml'
    )

    # Path to the simulator bringup launch file
    simulator_bringup_launch_path = os.path.join(
        get_package_share_directory('autodrive_f1tenth'),  # Replace with your simulator package name
        'launch',
        'simulator_bringup_headless.launch.py'
    )
    
    return LaunchDescription([
        # Static Transform: world -> map
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            output='screen'
        ),

        # Static Transform: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # Static Transform: odom -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),

        #### NODES ####

        # robot_localization ekf_node
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     parameters=[
        #         config_file_path  # Replace with the correct path to your parameters file
        #     ],
        #     output='screen'
        # ),
        Node(
            package='autodrive_iros_2024',
            executable='frame_republisher',
            name='frame_republisher',
            parameters=[{
                'frames': [
                    'left_encoder',
                    'right_encoder',
                    'imu',
                    'lidar',
                    'front_camera',
                    'front_left_wheel',
                    'front_right_wheel',
                    'rear_left_wheel',
                    'rear_right_wheel',
                ]
            }]
        ),

        # Include the simulator bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulator_bringup_launch_path),
            launch_arguments={}.items()
        ),
    ])
