from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=["/home/autodrive_devkit/ros2_ws/src/autodrive_iros_2024/config/robot_localization.yaml"],
            remappings=[
                ('/odometry/filtered', '/odom_filtered')
            ]
        )
    ])
