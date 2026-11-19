"""Run a separate RGB-D camera on the Museum robot."""
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


temporary = TemporaryDirectory(prefix='com260-rgbd-')


def generate_launch_description():
    sim = Path(get_package_share_directory('robot_sim_demo'))
    model = ET.parse(sim / 'models/turtlebot3_burger/model.sdf')
    sensor = model.find('.//sensor[@name="front_camera"]')
    sensor.set('type', 'rgbd_camera')
    sensor.find('update_rate').text = '30'
    sensor.find('topic').text = '/rgbd'
    camera = sensor.find('camera')
    camera.find('horizontal_fov').text = str(2 * math.atan(320 / 615))
    camera.find('image/width').text = '640'
    camera.find('image/height').text = '480'
    camera.find('clip/near').text = '0.1'
    camera.find('clip/far').text = '5.0'
    robot = Path(temporary.name) / 'robot.sdf'
    model.write(robot)
    return LaunchDescription([
        DeclareLaunchArgument('gz_partition', default_value='com260_rgbd'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim / 'launch/gazebo2.launch.py')),
            launch_arguments={'spawn_robot': 'false', 'drive': 'false', 'rviz': 'false',
                              'gz_partition': LaunchConfiguration('gz_partition')}.items()),
        TimerAction(period=3.0, actions=[Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-world', 'default', '-file', str(robot), '-name', 'turtlebot3_burger',
                       '-x', '0', '-y', '0', '-z', '0.010', '-Y', '0.7853981633974483'])]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': '/course-config/rgbd-bridge.yaml', 'use_sim_time': True}]),
        Node(package='image_transport', executable='republish', name='rgbd_color_transport',
             arguments=['raw', 'compressed'], parameters=[{'out.format': 'png', 'use_sim_time': True}],
             remappings=[('in', '/camera/color/image_raw'),
                         ('out/compressed', '/camera/color/image_raw/compressed')]),
        Node(package='image_transport', executable='republish', name='rgbd_depth_transport',
             arguments=['raw', 'compressedDepth'], parameters=[{'use_sim_time': True}],
             remappings=[('in', '/camera/aligned_depth_to_color/image_raw'),
                         ('out/compressedDepth', '/camera/aligned_depth_to_color/image_raw/compressedDepth')]),
        Node(package='rviz2', executable='rviz2', name='rgbd_rviz', output='screen',
             arguments=['-d', '/course-rviz/rgbd-slam.rviz'],
             parameters=[{'use_sim_time': True}]),
    ])
