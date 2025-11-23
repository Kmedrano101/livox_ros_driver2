#!/usr/bin/env python3
"""
Launch file for dual LiDAR calibration visualization

This launch file starts:
1. LiDAR TF calibration node (publishes static transforms)
2. RViz2 for visualization

Usage:
    ros2 launch livox_ros_driver2 lidar_calibration_viz.launch.py

Optional arguments:
    use_rviz:=false     # Disable RViz if you want to run it separately
    l1_y:=0.11          # Adjust L1 Y position (meters)
    l2_y:=-0.11         # Adjust L2 Y position (meters)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # L1 position arguments
    l1_x_arg = DeclareLaunchArgument('l1_x', default_value='0.0')
    l1_y_arg = DeclareLaunchArgument('l1_y', default_value='0.11')
    l1_z_arg = DeclareLaunchArgument('l1_z', default_value='0.0')
    l1_roll_arg = DeclareLaunchArgument('l1_roll', default_value='90.0')
    l1_pitch_arg = DeclareLaunchArgument('l1_pitch', default_value='0.0')
    l1_yaw_arg = DeclareLaunchArgument('l1_yaw', default_value='0.0')

    # L2 position arguments
    l2_x_arg = DeclareLaunchArgument('l2_x', default_value='0.0')
    l2_y_arg = DeclareLaunchArgument('l2_y', default_value='-0.11')
    l2_z_arg = DeclareLaunchArgument('l2_z', default_value='0.0')
    l2_roll_arg = DeclareLaunchArgument('l2_roll', default_value='90.0')
    l2_pitch_arg = DeclareLaunchArgument('l2_pitch', default_value='0.0')
    l2_yaw_arg = DeclareLaunchArgument('l2_yaw', default_value='180.0')

    # Get package share directory
    pkg_share = FindPackageShare('livox_ros_driver2')

    # Path to RViz config file
    rviz_config_file = PathJoinSubstitution(
        [pkg_share, 'rviz_cfg', 'lidar_calibration.rviz']
    )

    # LiDAR TF Calibration Node
    lidar_tf_node = Node(
        package='livox_ros_driver2',
        executable='lidar_tf_calibration.py',
        name='lidar_tf_calibration',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'l1_frame': 'lidar_L1',
            'l2_frame': 'lidar_L2',
            'l1_x': LaunchConfiguration('l1_x'),
            'l1_y': LaunchConfiguration('l1_y'),
            'l1_z': LaunchConfiguration('l1_z'),
            'l1_roll': LaunchConfiguration('l1_roll'),
            'l1_pitch': LaunchConfiguration('l1_pitch'),
            'l1_yaw': LaunchConfiguration('l1_yaw'),
            'l2_x': LaunchConfiguration('l2_x'),
            'l2_y': LaunchConfiguration('l2_y'),
            'l2_z': LaunchConfiguration('l2_z'),
            'l2_roll': LaunchConfiguration('l2_roll'),
            'l2_pitch': LaunchConfiguration('l2_pitch'),
            'l2_yaw': LaunchConfiguration('l2_yaw'),
        }]
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        l1_x_arg, l1_y_arg, l1_z_arg,
        l1_roll_arg, l1_pitch_arg, l1_yaw_arg,
        l2_x_arg, l2_y_arg, l2_z_arg,
        l2_roll_arg, l2_pitch_arg, l2_yaw_arg,

        # Nodes
        lidar_tf_node,
        rviz_node,
    ])
