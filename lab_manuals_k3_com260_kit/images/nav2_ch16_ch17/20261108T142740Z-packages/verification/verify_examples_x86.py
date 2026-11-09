import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
from contextlib import contextmanager

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

root = Path('/workspace')
out = root / 'runs/20261108T142740Z-packages/checks'
out.mkdir(parents=True, exist_ok=True)
packages = {'custom_nav_component': 'custom_nav_component', 'bt_navigator_client': 'navigation_client_cpp', 'multi_goal_navigation': 'navigation_client_cpp', 'global_planner_client': 'global_planner_client', 'costmap_demo': 'costmap_demo', 'inflation_plot': 'costmap_demo', 'map_loader': 'map_loader', 'obstacle_layer_processor': 'obstacle_layer_processor', 'costmap_debugger': 'costmap_debugger'}
def binary(name):
    return root / "install" / packages[name] / "lib" / packages[name] / name
print('RUN_ID=20261108T142740Z-packages', flush=True)
rclpy.init()
node = rclpy.create_node('chapter_16_17_checks')
results = []

def wait_for(predicate, seconds=15):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError('Timed out waiting for the expected result')

def check(name, condition):
    assert condition, name
    results.append(name)
    print('PASS', name, flush=True)

def call(client, request):
    assert client.wait_for_service(timeout_sec=10), client.srv_name
    future = client.call_async(request)
    wait_for(future.done)
    return future.result()

@contextmanager
def running(name):
    log = out / (name + '.log')
    with log.open('w') as stream:
        process = subprocess.Popen([str(binary(name))], stdout=stream, stderr=subprocess.STDOUT)
        try:
            yield log
        finally:
            process.send_signal(signal.SIGINT)
            try:
                code = process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise AssertionError(name + ' failed to stop')
            check(name + ' clean exit', code == 0)

try:
    with running('custom_nav_component') as log:
        change = node.create_client(ChangeState, '/custom_nav_component/change_state')
        state = node.create_client(GetState, '/custom_nav_component/get_state')
        check('lifecycle initially unconfigured', call(state, GetState.Request()).current_state.id == 1)
        scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        for transition, expected in [(1, 2), (3, 3), (4, 2), (2, 1), (1, 2), (3, 3), (7, 4)]:
            request = ChangeState.Request()
            request.transition.id = transition
            check(f'lifecycle transition {transition}', call(change, request).success)
            check(f'lifecycle state {expected} after {transition}', call(state, GetState.Request()).current_state.id == expected)
            if expected == 3:
                wait_for(lambda: scan_pub.get_subscription_count() > 0)
                msg = LaserScan()
                msg.header.frame_id = 'map'
                msg.ranges = [1.0, 2.0]
                scan_pub.publish(msg)
                wait_for(lambda: 'Active: scan=2' in log.read_text())
        check('lifecycle configure twice without duplicate parameter error', log.read_text().count('配置完成') == 2)
        node.destroy_publisher(scan_pub)
        node.destroy_client(change)
        node.destroy_client(state)

    with running('map_loader'):
        client = node.create_client(LoadMap, '/map_server/load_map')
        (out / 'grid.pgm').write_bytes(b'P5\n# fixture\n2 2\n255\n' + bytes([0, 255, 205, 128]))
        yaml_path = out / 'grid.yaml'
        text = 'image: grid.pgm\nresolution: 0.05\norigin: [1.0, 2.0, 1.5707963267948966]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
        yaml_path.write_text(text)
        request = LoadMap.Request(map_url=str(yaml_path))
        response = call(client, request)
        check('map load success', response.result == 0)
        check('PGM orientation, black/white and unknown', list(response.map.data) == [-1, -1, 100, 0])
        check('map yaw preserved', abs(response.map.info.origin.orientation.z - math.sqrt(0.5)) < 1e-6)
        maps = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = node.create_subscription(OccupancyGrid, '/map', lambda msg: maps.append(msg), qos)
        wait_for(lambda: maps)
        check('late map subscriber gets retained map', list(maps[-1].data) == [-1, -1, 100, 0])
        yaml_path.write_text(text.replace('negate: 0', 'negate: 1'))
        check('negated PGM conversion', list(call(client, request).map.data) == [100, -1, 0, 100])
        (out / 'grid.pgm').write_text('P2\n2 2\n255\n0 255\n205 128\n')
        yaml_path.write_text(text)
        check('P2 parsing', list(call(client, request).map.data) == [-1, -1, 100, 0])
        missing = call(client, LoadMap.Request(map_url=str(out / 'missing.yaml')))
        check('missing map rejected', missing.result != 0)
        yaml_path.write_text(text.replace('resolution: 0.05', 'resolution: .nan'))
        check('nonfinite resolution rejected', call(client, request).result != 0)
        node.destroy_subscription(sub)
        node.destroy_client(client)

    with running('obstacle_layer_processor'):
        maps = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = node.create_subscription(OccupancyGrid, '/teaching_obstacles', lambda msg: maps.append(msg), qos)
        scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        cloud_pub = node.create_publisher(PointCloud2, '/point_cloud', qos_profile_sensor_data)
        wait_for(lambda: scan_pub.get_subscription_count() > 0 and cloud_pub.get_subscription_count() > 0)
        def value(x, y):
            msg = maps[-1]
            gx = math.floor((x - msg.info.origin.position.x) / msg.info.resolution + 1e-5)
            gy = math.floor((y - msg.info.origin.position.y) / msg.info.resolution + 1e-5)
            return msg.data[gy * msg.info.width + gx]
        scan = LaserScan()
        scan.header.frame_id = 'map'
        scan.range_min, scan.range_max = 0.1, 5.0
        scan.angle_min, scan.angle_max, scan.angle_increment = 0.0, 0.0, 1.0
        scan.ranges = [1.0]
        scan_pub.publish(scan)
        wait_for(lambda: maps and value(1.01, 0.01) == 100)
        check('laser endpoint marking', value(1.01, 0.01) == 100)
        scan.ranges = [2.0]
        scan_pub.publish(scan)
        wait_for(lambda: value(2.01, 0.01) == 100)
        check('ray clears old obstacle', value(1.01, 0.01) == 0)
        cloud = PointCloud2()
        cloud.header.frame_id = 'map'
        cloud.height, cloud.width = 1, 3
        cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, name in enumerate('xyz')]
        cloud.point_step, cloud.row_step = 12, 36
        cloud.data = b''.join(struct.pack('<fff', *p) for p in [(0, 1, 0.5), (0, 2, 3), (float('nan'), 0, 0)])
        cloud_pub.publish(cloud)
        wait_for(lambda: value(0.01, 1.01) == 100)
        check('cloud valid height marked', value(0.01, 1.01) == 100)
        check('cloud high point filtered', value(0.01, 2.01) == 0)
        node.destroy_subscription(sub)
        node.destroy_publisher(scan_pub)
        node.destroy_publisher(cloud_pub)
    (out / 'results.json').write_text(json.dumps({'passed': True, 'checks': results}, ensure_ascii=False, indent=2))
    print('ALL_CHECKS_PASSED', len(results), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
