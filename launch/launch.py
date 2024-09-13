from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ### RTABMAP configuration ###

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    
    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':False,
          'subscribe_rgb':False,
          'subscribe_scan':True, # only use lidar for now
          'approx_sync':True,
          'use_action_for_goal':True,
          'qos_scan':qos,
          'qos_imu':qos,
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }
    
    remappings=[
          ('scan', '/autodrive/f1tenth_1/lidar')]


    ### AutoDrive simulator setup ### 
    config_file_path = os.path.join(
        get_package_share_directory('autodrive_iros_2024'),  # Replace with your package name
        'config', 
        'robot_localization.yaml'
    )

    # Path to the simulator bringup launch file
    simulator_bringup_launch_path = os.path.join(
        get_package_share_directory('autodrive_f1tenth'),  # Replace with your simulator package name
        'launch',
        # 'simulator_bringup_headless.launch.py'
        'simulator_bringup_rviz.launch.py'
    )
    
    return LaunchDescription([

        ### ARGUMENTS ###

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
            
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        ### TRANSFORMS ###

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
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[
                config_file_path  # Replace with the correct path to your parameters file
            ],
            output='screen'
        ),
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
        Node(
            package='autodrive_iros_2024',
            executable='ackermann_odometry',
            name='ackermann_odometry',
            parameters=[],
            output='screen'
        ),


        # rtabmap nodes

        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),


        # Include the simulator bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulator_bringup_launch_path),
            launch_arguments={}.items()
        ),
    ])
