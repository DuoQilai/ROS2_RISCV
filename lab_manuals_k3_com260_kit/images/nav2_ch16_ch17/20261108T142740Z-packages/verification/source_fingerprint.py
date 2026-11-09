import hashlib,json,sys
from pathlib import Path
root=Path(sys.argv[1]).expanduser()
packages=['custom_nav_component', 'navigation_client_cpp', 'global_planner_client', 'costmap_demo', 'map_loader', 'obstacle_layer_processor', 'costmap_debugger']
files=[f for p in packages for f in sorted((root/"src_k3_com260_kit"/p).rglob("*"))]
files += [root/'setup_course_k3_com260_kit.sh',root/'course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash', root/'course_support/k3_com260_kit/launch/nav2-teaching.launch.py']
files += [root/'src_k3_com260_kit/param_demo_cpp'/p for p in ['config/nav2_params.yaml','launch/combined_sim_nav.launch.py','maps/Software_Museum.yaml','maps/Software_Museum.pgm','rviz/navigation.rviz']]
files += [root/'src_k3_pico_itx/robot_sim_demo'/p for p in ['launch/gazebo2.launch.py','gui/museum.gui.config','worlds/museum.sdf','models/turtlebot3_burger/model.sdf','config/gazebo2_bridge.yaml']]
print(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()},indent=2,sort_keys=True))
