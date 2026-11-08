from contextlib import contextmanager
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from nav2_msgs.srv import LoadMap
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

root=Path.home()/'ros2_course_com260_ch16_ch17_ws'
out=root/'runs/20261108T142740Z-packages/live-checks'
out.mkdir(parents=True, exist_ok=True)
packages = {'custom_nav_component': 'custom_nav_component', 'bt_navigator_client': 'navigation_client_cpp', 'multi_goal_navigation': 'navigation_client_cpp', 'global_planner_client': 'global_planner_client', 'costmap_demo': 'costmap_demo', 'inflation_plot': 'costmap_demo', 'map_loader': 'map_loader', 'obstacle_layer_processor': 'obstacle_layer_processor', 'costmap_debugger': 'costmap_debugger'}
def binary(name):
    return root / "install" / packages[name] / "lib" / packages[name] / name
print('RUN_ID=20261108T142740Z-packages', flush=True)
rclpy.init()
node=rclpy.create_node('nav2_live_checks')
checks=[]

def check(name, value):
    assert value, name
    checks.append(name)
    print('PASS',name,flush=True)

def wait(predicate, timeout=20):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=0.1)
        if predicate(): return
    raise AssertionError('Timed out')

def call(kind,name,request):
    client=node.create_client(kind,name)
    assert client.wait_for_service(timeout_sec=10),name
    future=client.call_async(request)
    wait(future.done)
    result=future.result()
    node.destroy_client(client)
    return result

@contextmanager
def running(executable, arguments=()):
    with (out/(executable+'.log')).open('w') as log:
        process=subprocess.Popen([str(binary(executable)),*arguments],stdout=log,stderr=subprocess.STDOUT)
        try: yield
        finally:
            process.send_signal(signal.SIGINT)
            try: code=process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill();process.wait();raise AssertionError('Failed to stop '+executable)
            check(executable+' clean exit',code==0)

try:
    with running('map_loader',['--ros-args','-p','use_sim_time:=true','-r','/map:=/teaching_map','-r','/map_server/load_map:=/teaching/load_map']):
        path=Path.home()/'ROS2_RISCV_COM260_CH16_CH17/src_k3_com260_kit/param_demo_cpp/maps/Software_Museum.yaml'
        loaded=call(LoadMap,'/teaching/load_map',LoadMap.Request(map_url=str(path)))
        reference=call(GetMap,'/map_server/map',GetMap.Request()).map
        check('Museum map loaded',loaded.result==0)
        check('Museum map geometry equals Nav2',loaded.map.info.width==reference.info.width and loaded.map.info.height==reference.info.height and loaded.map.info.resolution==reference.info.resolution)
        check('Museum map pixels equal Nav2',loaded.map.data==reference.data)
        print('Museum dimensions',loaded.map.info.width,loaded.map.info.height,flush=True)
    with running('obstacle_layer_processor',['--ros-args','-p','use_sim_time:=true']):
        maps=[]
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub=node.create_subscription(OccupancyGrid,'/teaching_obstacles',lambda msg:maps.append(msg),qos)
        wait(lambda:len(maps)>=3)
        check('real LaserScan and TF produce teaching map',maps[-1].header.frame_id=='map' and maps[-1].data.count(100)>0)
        print('Teaching map messages',len(maps),'lethal',maps[-1].data.count(100),flush=True)
        node.destroy_subscription(sub)
    with running('costmap_debugger',['--ros-args','-p','use_sim_time:=true']):
        log=out/'costmap_debugger.log'
        wait(lambda:'/global_costmap/costmap' in log.read_text() and '/local_costmap/costmap' in log.read_text())
        check('C++ debugger receives both real costmaps',True)
        print(log.read_text(),flush=True)
    for name,args,env,expected in [
        ('invalid-planner',['global_planner_client','--ros-args','-p','use_sim_time:=true','-p','planner_id:=MissingPlanner'],None,1),
        ('negative-cancel',['bt_navigator_client','--ros-args','-p','cancel_after:=-1.0'],None,1),
        ('missing-server',['bt_navigator_client'],dict(os.environ,ROS_DOMAIN_ID='183'),1)]:
        result=subprocess.run([str(binary(args[0])),*args[1:]],capture_output=True,text=True,env=env,timeout=25)
        (out/(name+'.log')).write_text(result.stdout+result.stderr)
        print(result.stdout+result.stderr,flush=True)
        check(name+' expected failure',result.returncode==expected)
    (out/'results.json').write_text(json.dumps({'passed':True,'checks':checks},ensure_ascii=False,indent=2))
    print('LIVE_CHECKS_PASSED',len(checks),flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
