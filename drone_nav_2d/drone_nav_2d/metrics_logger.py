import json
import math
import os
from datetime import datetime
from typing import List, Optional

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class MetricsLogger(Node):
    def __init__(self) -> None:
        super().__init__('metrics_logger')

        self.declare_parameter('output_file', 'results/mission_metrics.json')
        self.output_file = str(self.get_parameter('output_file').value)

        self.start_time = self.get_clock().now()
        self.first_motion_time: Optional[float] = None
        self.last_trajectory: Optional[Path] = None

        self.replan_count = 0
        self.min_obstacle_distance = float('inf')
        self.success = False
        self.written = False

        self.create_subscription(Path, '/drone_trajectory', self._on_trajectory, 10)
        self.create_subscription(Bool, '/replan_event', self._on_replan, 10)
        self.create_subscription(Float32, '/min_obstacle_distance', self._on_min_distance, 10)
        self.create_subscription(Bool, '/mission_complete', self._on_mission_complete, 10)

        self.get_logger().info('Metrics logger initialized.')

    def _on_trajectory(self, msg: Path) -> None:
        self.last_trajectory = msg
        if self.first_motion_time is None and len(msg.poses) > 1:
            self.first_motion_time = self._now_seconds()

    def _on_replan(self, msg: Bool) -> None:
        if msg.data:
            self.replan_count += 1

    def _on_min_distance(self, msg: Float32) -> None:
        self.min_obstacle_distance = min(self.min_obstacle_distance, float(msg.data))

    def _on_mission_complete(self, msg: Bool) -> None:
        if msg.data:
            self.success = True
            self._finalize_and_write()

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _path_length(self, points: List) -> float:
        total = 0.0
        for i in range(1, len(points)):
            p0 = points[i - 1].pose.position
            p1 = points[i].pose.position
            total += math.dist((p0.x, p0.y, p0.z), (p1.x, p1.y, p1.z))
        return total

    def _smoothness(self, points: List) -> float:
        if len(points) < 3:
            return 0.0

        heading_changes = 0.0
        prev_heading: Optional[float] = None
        for i in range(1, len(points)):
            p0 = points[i - 1].pose.position
            p1 = points[i].pose.position
            heading = math.atan2(p1.y - p0.y, p1.x - p0.x)
            if prev_heading is not None:
                delta = (heading - prev_heading + math.pi) % (2.0 * math.pi) - math.pi
                heading_changes += abs(delta)
            prev_heading = heading

        return heading_changes

    def _finalize_and_write(self) -> None:
        if self.written:
            return

        now_sec = self._now_seconds()
        start_sec = self.start_time.nanoseconds / 1e9
        mission_time = now_sec - (self.first_motion_time if self.first_motion_time is not None else start_sec)

        points = self.last_trajectory.poses if self.last_trajectory is not None else []
        path_length = self._path_length(points)
        smoothness = self._smoothness(points)

        metrics = {
            'timestamp': datetime.utcnow().isoformat() + 'Z',
            'total_path_length_m': round(path_length, 4),
            'mission_completion_time_s': round(max(0.0, mission_time), 4),
            'replanning_events': int(self.replan_count),
            'minimum_obstacle_distance_m': round(float(self.min_obstacle_distance if math.isfinite(self.min_obstacle_distance) else -1.0), 4),
            'path_smoothness_score': round(smoothness, 4),
            'success': bool(self.success),
            'failure': not bool(self.success),
        }

        output_path = self.output_file
        if not os.path.isabs(output_path):
            output_path = os.path.join(os.getcwd(), output_path)

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(metrics, f, indent=2)

        self.written = True
        self.get_logger().info('Mission metrics saved: ' + output_path)
        self.get_logger().info(json.dumps(metrics, indent=2))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._finalize_and_write()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
