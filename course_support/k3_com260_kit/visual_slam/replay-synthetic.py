"""Replay calibrated camera/IMU inputs and export VINS odometry as TUM poses."""
import csv
import json
import time
from pathlib import Path

import cv2
import rclpy
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import Image, Imu, PointCloud

rclpy.init()
node = rclpy.create_node("synthetic_camera_and_imu")
camera = node.create_publisher(Image, "/cam0/image_raw", 10)
imu = node.create_publisher(Imu, "/imu0", 200)
output = Path("out/vins")
output.mkdir(parents=True, exist_ok=True)
stream = (output / "poses.txt").open("w", buffering=1)
counts = {"features": 0, "graph": 0, "poses": 0}


def observe(message):
    pose = message.pose.pose
    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    p, q = pose.position, pose.orientation
    print(f"{stamp:.9f}", p.x, p.y, p.z, q.x, q.y, q.z, q.w, file=stream)
    counts["poses"] += 1


def features(message):
    counts["features"] += 1


def graph(message):
    counts["graph"] += 1
    counts["last_graph_path_length"] = len(message.poses)


node.create_subscription(Odometry, "/vins_estimator/odometry", observe, 100)
node.create_subscription(PointCloud, "/feature_tracker/feature", features, 100)
node.create_subscription(NavPath, "/pose_graph/pose_graph_path", graph, 100)
try:
    deadline = time.monotonic() + 35
    while camera.get_subscription_count() < 2 or imu.get_subscription_count() < 1:
        if time.monotonic() > deadline:
            raise RuntimeError("Waiting for feature tracker, estimator and pose graph timed out.")
        rclpy.spin_once(node, timeout_sec=.1)
    started = time.monotonic()
    images = 0
    with Path("sequence/imu.csv").open() as source:
        for index, row in enumerate(csv.reader(source)):
            stamp, *values = map(float, row)
            message = Imu()
            message.header.frame_id = "imu"
            message.header.stamp.sec = int(stamp)
            message.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
            message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = values[:3]
            message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = values[3:]
            # Preserve sensor timestamps while playing at 0.4 times real time.
            scheduled = started + index/80
            while time.monotonic() < scheduled:
                rclpy.spin_once(node, timeout_sec=min(.005, max(0., scheduled-time.monotonic())))
            imu.publish(message)
            if index % 10 == 0:
                pixels = cv2.imread(f"sequence/images/{index//10:06d}.png", cv2.IMREAD_GRAYSCALE)
                picture = Image()
                picture.header.stamp = message.header.stamp
                picture.header.frame_id = "camera"
                picture.height, picture.width = pixels.shape
                picture.step = picture.width
                picture.encoding = "mono8"
                picture.data = pixels.tobytes()
                camera.publish(picture)
                images += 1
            rclpy.spin_once(node, timeout_sec=0.)
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    counts.update(images=images, imu=index+1)
    (output/"counts.json").write_text(json.dumps(counts, indent=2)+"\n")
    if not counts["poses"]:
        raise RuntimeError("VINS produced no optimized poses.")
    print(counts)
except KeyboardInterrupt:
    pass
finally:
    stream.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
