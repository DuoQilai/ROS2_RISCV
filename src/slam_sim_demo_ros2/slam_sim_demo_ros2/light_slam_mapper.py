#!/usr/bin/env python3
"""Lightweight raycast occupancy-grid mapper for the museum navigation demo.

Subscribes to /scan and the odom->laser TF chain, accumulates a 2D occupancy
grid and republishes it on /map together with a static identity map->odom TF.
Intended for simulation only, where odometry is drift-free, so map == odom.
"""

import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros


class LightSlamMapper(Node):
    def __init__(self) -> None:
        super().__init__("light_slam_mapper")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("extent", 12.0)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("laser_frame", "laser_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("publish_period", 1.0)

        self.resolution = float(self.get_parameter("resolution").value)
        self.extent = float(self.get_parameter("extent").value)
        laser_frame = str(self.get_parameter("laser_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        period = float(self.get_parameter("publish_period").value)

        side = int(2 * self.extent / self.resolution)
        self.grid = np.full((side, side), -1, dtype=np.int8)
        self.origin_x = -self.extent
        self.origin_y = -self.extent

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_map_tf()

        self.latest_pose = None
        self.scans_processed = 0
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self.handle_scan, 10
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 1, transient_local_qos=True)
        self.create_timer(period, self.publish_map)
        self.get_logger().info(
            f"light_slam_mapper ready: {side}x{side} @ {self.resolution}m"
        )

    def create_publisher(self, msg_type, topic, qos, transient_local_qos=False):
        if transient_local_qos:
            from rclpy.qos import DurabilityPolicy, QoSProfile

            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        return super().create_publisher(msg_type, topic, qos)

    def _publish_static_map_tf(self) -> None:
        from geometry_msgs.msg import Transform, TransformStamped

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = self.odom_frame
        msg.transform = Transform()
        msg.transform.translation.x = 0.0
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.0
        msg.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(msg)

    def handle_scan(self, scan: LaserScan) -> None:
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.odom_frame, scan.header.frame_id, rclpy.time.Time()
            )
        except Exception:
            return
        tr = tfm.transform.translation
        rot = tfm.transform.rotation
        yaw = math.atan2(
            2.0 * (rot.w * rot.z + rot.x * rot.y),
            1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z),
        )
        ox, oy, oyaw = tr.x, tr.y, yaw
        self.latest_pose = (ox, oy)

        ranges = np.array(scan.ranges, dtype=np.float64)
        ranges = np.nan_to_num(ranges, nan=scan.range_max, posinf=scan.range_max, neginf=0.0)
        valid = (ranges >= scan.range_min) & (ranges <= scan.range_max)
        if not valid.any():
            return
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        world_angles = oyaw + angles[valid]
        hit_ranges = ranges[valid]
        cos_a = np.cos(world_angles)
        sin_a = np.sin(world_angles)
        res = self.resolution
        max_steps = int(hit_ranges.max() / res) + 1
        steps = (hit_ranges / res).astype(int)
        j = np.arange(1, max_steps + 1)[None, :]
        mask = j <= steps[:, None]
        dist = j * res + np.zeros((steps.shape[0], 1), dtype=np.float64)
        free_x = (ox + cos_a[:, None] * dist)[mask]
        free_y = (oy + sin_a[:, None] * dist)[mask]
        self._scatter(free_x, free_y, 0)
        self._scatter(ox + cos_a * hit_ranges, oy + sin_a * hit_ranges, 100)
        self.scans_processed += 1

    def _scatter(self, xs: np.ndarray, ys: np.ndarray, value: int) -> None:
        col = ((xs - self.origin_x) / self.resolution).astype(np.int64)
        row = ((ys - self.origin_y) / self.resolution).astype(np.int64)
        ok = (row >= 0) & (row < self.grid.shape[0]) & (col >= 0) & (col < self.grid.shape[1])
        if ok.any():
            self.grid[row[ok], col[ok]] = value

    def _set(self, x: float, y: float, value: int) -> None:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if 0 <= row < self.grid.shape[0] and 0 <= col < self.grid.shape[1]:
            current = self.grid[row, col]
            if value == 100 or (current != 100 and value >= 0):
                self.grid[row, col] = value

    def publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.resolution
        msg.info.width = int(self.grid.shape[1])
        msg.info.height = int(self.grid.shape[0])
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.reshape(-1).tolist()
        self.map_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = LightSlamMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
