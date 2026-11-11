"""Compare global planners and local controllers in an isolated course simulation."""
from pathlib import Path
from tempfile import NamedTemporaryFile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    planner = LaunchConfiguration('planner').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    share = Path(get_package_share_directory('param_demo_cpp'))
    sim = Path(get_package_share_directory('robot_sim_demo'))
    lab_scene = LaunchConfiguration('scene').perform(context) == 'navigation_lab'
    world = '/course-config/navigation-lab.sdf' if lab_scene else str(sim / 'worlds/museum.sdf')
    map_file = '/course-config/navigation-lab-map.yaml' if lab_scene else str(share / 'maps/Software_Museum.yaml')
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    config = yaml.safe_load(Path('/course-config/planning.yaml').read_text())
    params = yaml.safe_load((share / 'config/nav2_params.yaml').read_text())
    # Keep real and synthetic scans in separate buffers; AMCL uses only real /scan.
    for name, plugin in [('local_costmap', 'voxel_layer'), ('global_costmap', 'obstacle_layer')]:
        layer = params[name][name]['ros__parameters'][plugin]
        layer['observation_sources'] = 'scan injected_scan'
        layer['injected_scan'] = {**layer['scan'], 'topic': '/scan_injected',
                                  'observation_persistence': 0.2}
    lab = Path(get_package_share_directory('navigation_lab_cpp'))
    params.update(yaml.safe_load((lab / 'config/navigation_lab.yaml').read_text()))
    params['planner_server']['ros__parameters']['GridBased'] = config['planners'][planner]
    control = params['controller_server']['ros__parameters']
    control['progress_checker']['required_movement_radius'] = 0.1
    control['general_goal_checker']['xy_goal_tolerance'] = 0.1
    control['FollowPath'] = config['controllers'][controller]
    if controller == 'mppi':
        control['controller_frequency'] = 10.0
    with NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as output:
        yaml.safe_dump(params, output)
        parameters = output.name
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim / 'launch/gazebo2.launch.py')),
            launch_arguments={'world': world, 'rviz': 'false', 'drive': 'false', 'use_sim_time': 'true',
                              'gz_partition': LaunchConfiguration('gz_partition')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2 / 'launch/bringup_launch.py')),
            launch_arguments={'map': map_file,
                              'params_file': parameters, 'use_sim_time': 'true',
                              'autostart': 'true', 'use_composition': 'False'}.items()),
        Node(package='rviz2', executable='rviz2', name='navigation_rviz', output='screen',
             arguments=['-d', str(share / 'rviz/navigation.rviz')],
             parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='museum', choices=['museum', 'navigation_lab']),
        DeclareLaunchArgument('planner', default_value='navfn', choices=['navfn', 'astar', 'smac2d', 'hybrid']),
        DeclareLaunchArgument('controller', default_value='dwb', choices=['dwb', 'rpp', 'mppi']),
        DeclareLaunchArgument('gz_partition', default_value='com260_ch18_ch19'),
        OpaqueFunction(function=start),
    ])
