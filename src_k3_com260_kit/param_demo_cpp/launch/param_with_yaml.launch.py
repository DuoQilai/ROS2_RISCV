#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('param_demo_cpp'),
        'config',
        'params.yaml',
    )

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='param_demo 节点使用的 YAML 参数文件',
        ),

        Node(
            package='param_demo_cpp',
            executable='param_node',
            name='param_demo',
            output='screen',
            parameters=[params_file],
        ),
    ])
