import json
from pathlib import Path
import signal
import subprocess
import threading
import time
import rclpy
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

root=Path.home()/'ros2_course_com260_ch16_ch17_ws'
out=root/'runs/20261108T142740Z-packages/trees'
out.mkdir(parents=True, exist_ok=True)
print('RUN_ID=20261108T142740Z-packages', flush=True)
rclpy.init()
node=rclpy.create_node('behavior_tree_checks')
battery=node.create_publisher(BatteryState,'/battery_status',10)
level=0.8
def publish_battery():
    msg=BatteryState();msg.percentage=level;battery.publish(msg)
timer=node.create_timer(0.2,publish_battery)
odom=[]
node.create_subscription(Odometry,'/odom',lambda m:odom.append(m),qos_profile_sensor_data)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=0.05)
def run(name,tree,x,y,expected):
    path='/workspace/install/navigation_client_cpp/share/navigation_client_cpp/behavior_trees/'+tree
    command=[str(root/'install/navigation_client_cpp/lib/navigation_client_cpp/bt_navigator_client'),'--ros-args','-p','use_sim_time:=true','-p',f'x:={x}','-p',f'y:={y}','-p',f'behavior_tree:={path}']
    before=len(odom);start=time.monotonic()
    with (out/(name+'.log')).open('w') as log:
        process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        def copy():
            for line in process.stdout:log.write(line);log.flush();print(line,end='',flush=True)
        reader=threading.Thread(target=copy);reader.start()
        while process.poll() is None and time.monotonic()-start<205:spin(0.1)
        if process.poll() is None:process.send_signal(signal.SIGINT);spin(12)
        if process.poll() is None:process.kill()
        code=process.wait();reader.join(5)
    spin(3)
    samples=odom[before:]
    stopped=bool(samples) and all(abs(m.twist.twist.linear.x)<0.02 and abs(m.twist.twist.angular.z)<0.03 for m in samples[-20:])
    result={'case':name,'exit':code,'stopped':stopped,'odom_samples':len(samples),'duration_s':time.monotonic()-start,'passed':code==expected and stopped}
    print(json.dumps(result),flush=True)
    (out/(name+'.json')).write_text(json.dumps(result,indent=2))
    assert result['passed'],name
try:
    spin(2)
    run('teaching-tree','teaching_navigation.xml',1.0,0.0,0)
    run('custom-high-battery','custom_navigation.xml',0.0,-1.0,0)
    level=0.1;spin(3)
    run('custom-low-battery','custom_navigation.xml',1.0,-1.0,1)
finally:
    node.destroy_node();rclpy.shutdown()
