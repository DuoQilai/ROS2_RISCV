"""Drive the differential-drive robot on a constant-radius circle."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CircleDriver(Node):
    def __init__(self) -> None:
        super().__init__("circle_driver")
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("duration", 15.0)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.duration = max(float(self.get_parameter("duration").value), 0.0)
        self.started_at = time.monotonic()
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.05, self.publish_command)

        self.get_logger().info(
            "Circle driver ready: "
            f"linear={self.linear_speed:.2f} m/s, "
            f"angular={self.angular_speed:.2f} rad/s, "
            f"duration={self.duration:.1f} s"
        )

    def publish_command(self) -> None:
        if self.duration and time.monotonic() - self.started_at >= self.duration:
            self.stop()
            self.timer.cancel()
            return

        command = Twist()
        command.linear.x = self.linear_speed
        command.angular.z = self.angular_speed
        self.publisher.publish(command)

    def stop(self) -> None:
        if rclpy.ok():
            self.publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CircleDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
