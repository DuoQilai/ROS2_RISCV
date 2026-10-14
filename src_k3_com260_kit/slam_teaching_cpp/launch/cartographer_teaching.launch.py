import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    configuration_directory = os.path.join(
        get_package_share_directory('slam_lab'), 'config', 'cartographer')
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', 'burger_2d.lua',
            ],
            parameters=[{'use_sim_time': True}],
            remappings=[('scan', '/scan'), ('odom', '/odom')],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
            parameters=[{'use_sim_time': True}],
        ),
    ])
