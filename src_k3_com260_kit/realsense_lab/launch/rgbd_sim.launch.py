"""Add RGB-D and IMU sensors to a private copy of the Museum robot."""
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


temporary = TemporaryDirectory(prefix='com260-acquisition-')


def start(context):
    sim = Path(get_package_share_directory('robot_sim_demo'))
    share = Path(get_package_share_directory('realsense_lab'))
    rviz_config = yaml.safe_load((share / 'rviz/acquisition.rviz').read_text())
    if LaunchConfiguration('pointcloud').perform(context) == 'false':
        displays = rviz_config['Visualization Manager']['Displays']
        rviz_config['Visualization Manager']['Displays'] = [d for d in displays if d['Class'] != 'rviz_default_plugins/PointCloud2']
    rviz_file = Path(temporary.name) / 'acquisition.rviz'
    rviz_file.write_text(yaml.safe_dump(rviz_config))
    width, height, rate = map(int, LaunchConfiguration('profile').perform(context).split('x'))
    model = ET.parse(sim / 'models/turtlebot3_burger/model.sdf')
    sensor = model.find('.//sensor[@name="front_camera"]')
    sensor.set('type', 'rgbd_camera')
    sensor.find('update_rate').text = str(rate)
    sensor.find('topic').text = '/rgbd'
    camera = sensor.find('camera')
    camera.find('horizontal_fov').text = str(2 * math.atan(320 / 615))
    camera.find('image/width').text = str(width)
    camera.find('image/height').text = str(height)
    camera.find('clip/near').text = '0.1'
    camera.find('clip/far').text = '5.0'
    model.find('.//link[@name="base_link"]').append(ET.fromstring('''
      <sensor name="imu" type="imu"><always_on>true</always_on>
        <update_rate>200</update_rate><topic>/imu</topic><gz_frame_id>base_link</gz_frame_id>
      </sensor>'''))
    robot = Path(temporary.name) / 'robot.sdf'
    model.write(robot)
    world = ET.parse(sim / 'worlds/museum.sdf')
    world.find('world').append(ET.Element('plugin', filename='gz-sim-imu-system',
                                        name='gz::sim::systems::Imu'))
    world_file = Path(temporary.name) / 'museum.sdf'
    world.write(world_file)
    mappings = [
        ('/camera/color/image_raw', '/rgbd/image', 'Image', 'Image'),
        ('/camera/depth/image_rect_raw', '/rgbd/depth_image', 'Image', 'Image'),
        ('/camera/color/camera_info', '/rgbd/camera_info', 'CameraInfo', 'CameraInfo'),
        ('/camera/depth/camera_info', '/rgbd/camera_info', 'CameraInfo', 'CameraInfo'),
        ('/imu', '/imu', 'Imu', 'IMU'),
    ]
    if LaunchConfiguration('pointcloud').perform(context) == 'true':
        mappings += [('/camera/color/points', '/rgbd/points', 'PointCloud2', 'PointCloudPacked'),
                     ('/camera/aligned_depth_to_color/image_raw', '/rgbd/depth_image', 'Image', 'Image')]
    bridges = []
    for name, kinds in [('camera', ('Image', 'CameraInfo')), ('pointcloud', ('PointCloud2',)), ('imu', ('Imu',))]:
        entries = [dict(ros_topic_name=ros, gz_topic_name=gz, ros_type_name=f'sensor_msgs/msg/{rt}',
                        gz_type_name=f'gz.msgs.{gt}', direction='GZ_TO_ROS', publisher_queue=2)
                   for ros, gz, rt, gt in mappings if rt in kinds]
        if not entries:
            continue
        config = Path(temporary.name) / f'{name}.yaml'
        config.write_text(yaml.safe_dump(entries))
        parameters = {'config_file': str(config), 'use_sim_time': True}
        for entry in entries:
            if entry['ros_type_name'].endswith('PointCloud2'):
                parameters[f"qos_overrides.{entry['ros_topic_name']}.publisher.reliability"] = 'best_effort'
        bridges.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name=f'{name}_bridge',
                            parameters=[parameters], output='screen'))
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim / 'launch/gazebo2.launch.py')),
            launch_arguments={'spawn_robot': 'false', 'world': str(world_file), 'drive': 'false',
                              'rviz': 'false', 'gz_partition': LaunchConfiguration('gz_partition')}.items()),
        TimerAction(period=3.0, actions=[Node(package='ros_gz_sim', executable='create',
            arguments=['-world', 'default', '-file', str(robot), '-name', 'turtlebot3_burger',
                       '-x', '0', '-y', '0', '-z', '0.010'], output='screen')]),
        *bridges,
        Node(package='image_transport', executable='republish', name='rgbd_color_transport',
             arguments=['raw', 'compressed'], parameters=[{'out.format': 'png', 'use_sim_time': True}],
             remappings=[('in', '/camera/color/image_raw'),
                         ('out/compressed', '/camera/color/image_raw/compressed')]),
        Node(package='rviz2', executable='rviz2', name='acquisition_rviz', output='screen',
             arguments=['-d', str(rviz_file)], parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('GZ_TRANSPORT_RCVHWM', '10'),
        SetEnvironmentVariable('GZ_TRANSPORT_SNDHWM', '10'),
        DeclareLaunchArgument('pointcloud', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('profile', default_value='640x480x30'),
        DeclareLaunchArgument('gz_partition', default_value='com260_rgbd_acquisition'),
        OpaqueFunction(function=start),
    ])
