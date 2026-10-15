from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_teaching_cpp',
            executable='hector_slam',
            name='hector_slam',
            output='screen',
            parameters=[{
                'map_resolution': 0.05,
                'map_size': 20.0,
                'max_iterations': 15,
                'update_factor_free': 0.4,
                'update_factor_occupied': 0.9,
                'use_sim_time': True,
            }],
        ),
    ])
