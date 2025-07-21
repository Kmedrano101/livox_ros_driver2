#!/usr/bin/env python3

__author__ = "Kevin Medrano Ayala"
__contact__ = "kevin.ejem18@gmail.com"

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('livox_ros_driver2')
    param_file = os.path.join(pkg_share, 'config', 'livox_params.yaml')
    user_config_path = os.path.join(pkg_share, 'config', 'multiple_netconfigs.json')

    params_override = {
        'user_config_path': user_config_path,
    }

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[param_file, params_override]
    )

    return LaunchDescription([
        livox_driver
    ])