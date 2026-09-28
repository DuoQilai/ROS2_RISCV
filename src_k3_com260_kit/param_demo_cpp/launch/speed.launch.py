#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('param_demo_cpp'),
        'config',
        'speed_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='param_demo_cpp',
            executable='speed_ctrl',
            name='speed_controller',
            output='screen',
            parameters=[params_file],
        ),
    ])
