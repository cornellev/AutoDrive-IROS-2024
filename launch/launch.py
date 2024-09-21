from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_localization_config_path = os.path.join(
        get_package_share_directory("autodrive_iros_2024"),
        "config",
        "robot_localization.yaml",
    )

    actuator_config_path = os.path.join(
        get_package_share_directory("autodrive_iros_2024"), "config", "actuator.yaml"
    )

    # Path to the simulator bringup launch file
    simulator_bringup_launch_path = os.path.join(
        get_package_share_directory("autodrive_f1tenth"),
        "launch",
        # "simulator_bringup_headless.launch.py",
        "simulator_bringup_rviz.launch.py",
    )

    return LaunchDescription(
        [
            # Static Transform: world -> map
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_world_to_map",
                arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
            ),
            # Static Transform: map -> odom
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_map_to_odom",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),
            # Static Transform: odom -> base_link
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_odom_to_base_link",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            ),
            Node(
                package="autodrive_iros_2024",
                executable="sensor_republisher",
                name="sensor_republisher",
                output="screen",
            ),
            Node(
                package="autodrive_iros_2024",
                executable="frame_republisher",
                name="frame_republisher",
                parameters=[
                    {
                        "frames": [
                            "left_encoder",
                            "right_encoder",
                            "imu",
                            "lidar",
                            "front_camera",
                            "front_left_wheel",
                            "front_right_wheel",
                            "rear_left_wheel",
                            "rear_right_wheel",
                        ]
                    }
                ],
            ),
            #### NODES ####
            # robot_localization ekf_node
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[
                    robot_localization_config_path  # Replace with the correct path to your parameters file
                ],
            ),
            Node(
                package="autodrive_iros_2024",
                executable="ackermann_odometry",
                name="ackermann_odometry",
                parameters=[],
            ),
            Node(
                package="autodrive_iros_2024",
                executable="actuator_control",
                name="actuator_control_node",
                parameters=[actuator_config_path],
                output="screen",
            ),
            Node(
                package="autodrive_iros_2024",
                executable="simple_driver",
                name="simple_driver",
                parameters=[],
                output="screen",
            ),
            # SLAM
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="async_slam_toolbox_node",
                parameters=[
                    {
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "map_frame": "map",
                        "scan_topic": "/scan",
                        "scan_queue_size": 1,
                        "map_update_interval": 0.3,
                    }
                ],
                output="screen",
            ),
            # Include the simulator bringup launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulator_bringup_launch_path),
                launch_arguments={}.items(),
            ),
        ]
    )
