"""Run three namespaced Nav2 robots in the course navigation world."""
from copy import deepcopy
from pathlib import Path
from tempfile import mkdtemp
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    config = Path("/course-config")
    generated = Path(mkdtemp(prefix="fleet-"))
    sim = Path(share("robot_sim_demo"))
    nav2 = Path(share("nav2_bringup"))
    world = ET.parse(config / "navigation-lab.sdf")
    scene = world.find("world")
    scene.find("physics/max_step_size").text = "0.002"
    model = ET.parse(sim / "models/turtlebot3_burger/model.sdf").find("model")
    bridge = yaml.safe_load((sim / "config/gazebo2_bridge.yaml").read_text())
    parameters = yaml.safe_load((Path(share("param_demo_cpp")) / "config/nav2_params.yaml").read_text())
    topics = [bridge[0]]
    actions = []
    description = (sim / "urdf/turtlebot3_burger.urdf").read_text()
    for index, y in enumerate([-1., 0., 1.], 1):
        name = f"robot{index}"
        body = deepcopy(model)
        body.set("name", name)
        ET.SubElement(body, "pose").text = f"-1.4 {y} 0.015 0 0 0"
        for link in body.findall("link"):
            for camera in link.findall("sensor[@type='camera']"):
                link.remove(camera)
        for tag in ["topic", "odom_topic", "tf_topic"]:
            for element in body.iter(tag):
                element.text = "/" + name + element.text
        scene.append(body)
        for entry in bridge[1:]:
            names = {key: "/" + name + entry[key] for key in ["gz_topic_name", "ros_topic_name"]}
            topics.append({**entry, **names})
        params = deepcopy(parameters)
        params["amcl"]["ros__parameters"]["initial_pose"].update(x=-1.4, y=y)
        params["bt_navigator"]["ros__parameters"]["odom_topic"] = "odom"
        control = params["controller_server"]["ros__parameters"]
        control["FollowPath"]["xy_goal_tolerance"] = .10
        control["general_goal_checker"]["xy_goal_tolerance"] = .12
        control["progress_checker"]["required_movement_radius"] = .10
        for scope, layer in [("local_costmap", "voxel_layer"), ("global_costmap", "obstacle_layer")]:
            costmap = params[scope][scope]["ros__parameters"]
            costmap[layer]["scan"]["topic"] = f"/{name}/scan"
            costmap["static_layer"]["map_topic"] = f"/{name}/map"
        parameter_file = generated / f"{name}.yaml"
        parameter_file.write_text(yaml.safe_dump(params))
        state = Node(
            package="robot_state_publisher", executable="robot_state_publisher", namespace=name,
            parameters=[{"robot_description": description, "use_sim_time": True}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")])
        navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2 / "launch/bringup_launch.py")),
            launch_arguments={
                "namespace": name, "use_namespace": "True", "use_composition": "True",
                "params_file": str(parameter_file), "map": str(config / "navigation-lab-map.yaml"),
                "use_sim_time": "True", "autostart": "True",
            }.items())
        actions.append(GroupAction([state, navigation]))
    world.write(generated / "fleet.sdf")
    (generated / "bridge.yaml").write_text(yaml.safe_dump(topics))
    gui = (sim / "gui/museum.gui.config").read_text()
    gui = gui.replace("<camera_pose>0 0 3.0 0 1.5708 0</camera_pose>",
                      "<camera_pose>-0.45 0 5.2 0 1.5708 0</camera_pose>")
    (generated / "fleet.gui.config").write_text(gui)
    return LaunchDescription([
        ExecuteProcess(
            cmd=["gz", "sim", "-r", "--gui-config", str(generated / "fleet.gui.config"),
                 str(generated / "fleet.sdf")],
            additional_env={"GZ_SIM_RESOURCE_PATH": str(sim / "models")}, output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             parameters=[{"config_file": str(generated / "bridge.yaml")}]),
        *actions,
    ])
