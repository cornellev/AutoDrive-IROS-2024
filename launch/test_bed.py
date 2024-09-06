from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import PushRosNamespace, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Path to the other launch file
    other_launch_file = os.path.join(
        FindPackageShare('other_package').find('other_package'),
        'launch',
        'other_launch.py'
    )

    return LaunchDescription([
        # Declare launch arguments (optional)
        DeclareLaunchArgument('param_name', default_value='default_value', description='A parameter'),

        # Include the other launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(other_launch_file),
            launch_arguments={'param_name': LaunchConfiguration('param_name')}.items()
        ),
        
        # You can also launch additional nodes here if needed
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
