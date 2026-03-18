import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class ObstacleAvoidance(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_avoidance')

        self.declare_parameter('scan_topic', '/webots/drone/scan')
        self.declare_parameter('obstacle_threshold_m', 0.5)
        self.declare_parameter('influence_distance_m', 1.2)
        self.declare_parameter('repulsive_gain', 0.9)
        self.declare_parameter('tangential_gain', 0.45)
        self.declare_parameter('max_avoidance_velocity', 0.8)

        scan_topic = self.get_parameter('scan_topic').value
        self.obstacle_threshold = float(self.get_parameter('obstacle_threshold_m').value)
        self.influence_distance = float(self.get_parameter('influence_distance_m').value)
        self.repulsive_gain = float(self.get_parameter('repulsive_gain').value)
        self.tangential_gain = float(self.get_parameter('tangential_gain').value)
        self.max_avoidance_velocity = float(self.get_parameter('max_avoidance_velocity').value)

        self.last_scan: Optional[LaserScan] = None
        self.obstacle_active = False

        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)

        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.avoidance_pub = self.create_publisher(Twist, '/avoidance_cmd_vel', 10)
        self.replan_pub = self.create_publisher(Bool, '/replan_request', 10)
        self.min_distance_pub = self.create_publisher(Float32, '/min_obstacle_distance', 10)

        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info('Obstacle avoidance initialized.')

    def _on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def _tick(self) -> None:
        if self.last_scan is None:
            self._publish_state(False, Twist(), float('inf'))
            return

        ranges = [r for r in self.last_scan.ranges if math.isfinite(r)]
        if not ranges:
            self._publish_state(False, Twist(), float('inf'))
            return

        min_dist = min(ranges)
        obstacle_detected = min_dist < self.obstacle_threshold

        cmd = Twist()
        if obstacle_detected:
            rep_x, rep_y = self._compute_potential_field(self.last_scan)
            cmd.linear.x = rep_x
            cmd.linear.y = rep_y
            speed = math.hypot(cmd.linear.x, cmd.linear.y)
            if speed > self.max_avoidance_velocity and speed > 0.0:
                scale = self.max_avoidance_velocity / speed
                cmd.linear.x *= scale
                cmd.linear.y *= scale

        self._publish_state(obstacle_detected, cmd, min_dist)

    def _compute_potential_field(self, scan: LaserScan) -> tuple[float, float]:
        rep_x = 0.0
        rep_y = 0.0

        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance > self.influence_distance:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            strength = self.repulsive_gain * (1.0 / max(distance, 0.05) - 1.0 / self.influence_distance)
            strength = max(0.0, strength)

            rep_x -= strength * math.cos(angle)
            rep_y -= strength * math.sin(angle)

            tangential = self.tangential_gain * strength
            rep_x += -tangential * math.sin(angle)
            rep_y += tangential * math.cos(angle)

        return rep_x, rep_y

    def _publish_state(self, detected: bool, cmd: Twist, min_dist: float) -> None:
        obstacle_msg = Bool()
        obstacle_msg.data = detected
        self.obstacle_pub.publish(obstacle_msg)

        self.avoidance_pub.publish(cmd)

        dist_msg = Float32()
        dist_msg.data = float(min_dist if math.isfinite(min_dist) else 999.0)
        self.min_distance_pub.publish(dist_msg)

        if detected and not self.obstacle_active:
            repl = Bool()
            repl.data = True
            self.replan_pub.publish(repl)

        self.obstacle_active = detected


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
