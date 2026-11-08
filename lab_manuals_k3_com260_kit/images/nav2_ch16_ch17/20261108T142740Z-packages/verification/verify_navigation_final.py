import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time
import threading
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.srv import GetCostmap
from rclpy.qos import qos_profile_sensor_data

root = Path.home() / 'ros2_course_com260_ch16_ch17_ws'
out = root / 'runs/20261108T142740Z-packages/navigation'
out.mkdir(parents=True, exist_ok=True)
print('RUN_ID=20261108T142740Z-packages', flush=True)
rclpy.init()
node = rclpy.create_node('navigation_acceptance')
buffer = Buffer(); listener = TransformListener(buffer, node)
odom, poses, commands = [], [], []
node.create_subscription(Odometry, '/odom', lambda m: odom.append((time.monotonic(), m)), qos_profile_sensor_data)
node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: poses.append((time.monotonic(), m)), 10)
node.create_subscription(Twist, '/cmd_vel', lambda m: commands.append((time.monotonic(), m)), 10)

def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

def run(name, executable, parameters, expected, target=None, stop_signal=None, limit=200):
    spin(2)
    start = time.monotonic()
    count = len(odom)
    command = [str(root / 'install/navigation_client_cpp/lib/navigation_client_cpp' / executable), '--ros-args', '-p', 'use_sim_time:=true']
    for key, value in parameters.items():
        command += ['-p', f'{key}:={value}']
    successes, arrivals = [], []
    targets = [(1,0),(1,-1),(2,-1),(0,-1),(0,0)] if name == 'five-goals' else []
    def capture_arrivals():
        if len(arrivals) < len(successes) and len(arrivals) < len(targets) and time.monotonic()-successes[len(arrivals)] >= 1:
            transform = buffer.lookup_transform('map', 'base_link', Time()).transform.translation
            tx, ty = targets[len(arrivals)]
            recent = [m for stamp,m in odom if stamp > time.monotonic()-0.5]
            arrivals.append({'target':[tx,ty], 'map_base_xy':[transform.x,transform.y],
                             'position_error_m':math.hypot(transform.x-tx,transform.y-ty),
                             'stopped':bool(recent) and all(abs(m.twist.twist.linear.x)<0.02 and abs(m.twist.twist.angular.z)<0.03 for m in recent)})
    with (out / (name + '.log')).open('w') as log:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        def copy_output():
            for line in process.stdout:
                if '导航结果: SUCCEEDED' in line: successes.append(time.monotonic())
                log.write(line)
                log.flush()
                print(line, end='', flush=True)
        reader = threading.Thread(target=copy_output)
        reader.start()
        sent_signal = False
        while process.poll() is None and time.monotonic() - start < limit:
            rclpy.spin_once(node, timeout_sec=0.05)
            capture_arrivals()
            if stop_signal and not sent_signal and time.monotonic() - start > 6:
                process.send_signal(stop_signal)
                sent_signal = True
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            spin(10)
        if process.poll() is None:
            process.kill()
        code = process.wait()
        reader.join(timeout=5)
    spin(3)
    capture_arrivals()
    samples = odom[count:]
    assert samples, 'No real odometry received'
    motion = [(m.pose.pose.position.x, m.pose.pose.position.y) for _, m in samples]
    distance = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(motion, motion[1:]))
    recent = [m for stamp, m in samples if stamp > time.monotonic()-2]
    stopped = bool(recent) and all(abs(m.twist.twist.linear.x) < 0.02 and abs(m.twist.twist.angular.z) < 0.03 for m in recent)
    pose = poses[-1][1].pose.pose if poses else None
    final_tf = buffer.lookup_transform('map', 'base_link', Time()).transform.translation
    error = math.hypot(final_tf.x-target[0], final_tf.y-target[1]) if target else None
    text = (out / (name + '.log')).read_text()
    result = {'name': name, 'exit': code, 'duration_s': time.monotonic()-start, 'odom_samples': len(samples),
              'distance_m': distance, 'stopped': stopped, 'position_error_m': error,
              'feedback_lines': text.count('剩余距离'), 'result_lines': [line for line in text.splitlines() if '导航结果' in line],
              'amcl_xy': [pose.position.x, pose.position.y] if pose else None,
              'map_base_xy': [final_tf.x, final_tf.y], 'position_source': 'latest map -> base_link TF after stop',
              'passed': code == expected and stopped and (error is None or error <= 0.25)}
    if targets:
        result['arrivals'] = arrivals
        result['passed'] = result['passed'] and len(arrivals)==5 and all(a['position_error_m']<=0.25 and a['stopped'] for a in arrivals)
    (out / (name + '.json')).write_text(json.dumps(result, ensure_ascii=False, indent=2))
    (out / (name + '-odom.csv')).write_text('wall_time,x,y,vx,wz\n' + ''.join(
        f'{stamp},{m.pose.pose.position.x},{m.pose.pose.position.y},{m.twist.twist.linear.x},{m.twist.twist.angular.z}\n' for stamp,m in samples))
    print(json.dumps(result, ensure_ascii=False), flush=True)
    assert result['passed'], name

try:
    mode = sys.argv[1]
    if mode == 'free':
        run('free-goal-1-0', 'bt_navigator_client', {'x':1.0,'y':0.0}, 0, (1,0))
    elif mode == 'costs':
        client = node.create_client(GetCostmap, '/global_costmap/get_costmap')
        assert client.wait_for_service(timeout_sec=15)
        future = client.call_async(GetCostmap.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        grid = future.result().map
        meta = grid.metadata
        for x,y in [(0,0),(1,0),(2,0),(2,1),(1,1),(3,2),(0,-1),(1,-1),(2,-1)]:
            gx = int((x-meta.origin.position.x)/meta.resolution)
            gy = int((y-meta.origin.position.y)/meta.resolution)
            print(x, y, grid.data[gy*meta.size_x+gx], flush=True)
    elif mode == 'goal':
        run('goal-3-2', 'bt_navigator_client', {}, 0, (3,2))
    elif mode == 'home':
        run('home', 'bt_navigator_client', {'x':0.0,'y':0.0}, 0, (0,0))
    elif mode == 'multi':
        run('five-goals', 'multi_goal_navigation', {}, 0, (0,0), limit=600)
    elif mode == 'cancel':
        run('cancel', 'bt_navigator_client', {'x':3.0,'y':2.0,'cancel_after':6.0}, 1)
    elif mode == 'signal':
        run('sigint', 'bt_navigator_client', {'x':3.0,'y':2.0}, 1, stop_signal=signal.SIGINT)
        run('sigterm', 'bt_navigator_client', {'x':3.0,'y':2.0}, 1, stop_signal=signal.SIGTERM)
    else:
        raise ValueError(mode)
finally:
    node.destroy_node()
    rclpy.shutdown()
