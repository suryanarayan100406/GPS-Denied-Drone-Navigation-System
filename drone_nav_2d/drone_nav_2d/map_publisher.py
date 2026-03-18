import math
from typing import List, Dict

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from scipy.ndimage import binary_dilation
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_OBSTACLES: List[Dict[str, float]] = [
    {'type': 'box', 'x': -2.8, 'y': 1.6, 'sx': 0.6, 'sy': 0.8},
    {'type': 'box', 'x': -2.2, 'y': -1.5, 'sx': 0.8, 'sy': 0.6},
    {'type': 'box', 'x': -0.8, 'y': 0.8, 'sx': 1.0, 'sy': 0.5},
    {'type': 'box', 'x': 0.5, 'y': -1.8, 'sx': 0.6, 'sy': 1.0},
    {'type': 'box', 'x': 1.6, 'y': 1.7, 'sx': 0.7, 'sy': 0.7},
    {'type': 'cylinder', 'x': -0.2, 'y': -0.3, 'r': 0.35},
    {'type': 'cylinder', 'x': 2.4, 'y': -0.2, 'r': 0.4},
    {'type': 'cylinder', 'x': 3.0, 'y': 1.3, 'r': 0.3},
    {'type': 'cylinder', 'x': 0.0, 'y': 2.5, 'r': 0.35},
    {'type': 'cylinder', 'x': -3.2, 'y': -2.5, 'r': 0.3},
]


class MapPublisher(Node):
    def __init__(self) -> None:
        super().__init__('map_publisher')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('width', 100)
        self.declare_parameter('height', 100)
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('inflation_radius_m', 0.4)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.inflation_radius_m = self.get_parameter('inflation_radius_m').get_parameter_value().double_value

        self.origin_x = -0.5 * self.width * self.resolution
        self.origin_y = -0.5 * self.height * self.resolution

        self.grid_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)

        self._occupancy = self._build_occupancy_grid()
        self._timer = self.create_timer(max(0.05, 1.0 / publish_rate_hz), self._publish)

        self.get_logger().info('Map publisher initialized.')

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def _build_occupancy_grid(self) -> np.ndarray:
        grid = np.zeros((self.height, self.width), dtype=np.uint8)

        for obs in DEFAULT_OBSTACLES:
            if obs['type'] == 'box':
                self._paint_box(grid, obs['x'], obs['y'], obs['sx'], obs['sy'])
            elif obs['type'] == 'cylinder':
                self._paint_cylinder(grid, obs['x'], obs['y'], obs['r'])

        inflation_cells = max(1, int(math.ceil(self.inflation_radius_m / self.resolution)))
        kernel_size = inflation_cells * 2 + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=bool)
        inflated = binary_dilation(grid.astype(bool), structure=kernel)
        return (inflated.astype(np.uint8) * 100).astype(np.int8)

    def _paint_box(self, grid: np.ndarray, cx: float, cy: float, sx: float, sy: float) -> None:
        min_x, min_y = self.world_to_grid(cx - sx * 0.5, cy - sy * 0.5)
        max_x, max_y = self.world_to_grid(cx + sx * 0.5, cy + sy * 0.5)

        min_x = max(0, min(self.width - 1, min_x))
        max_x = max(0, min(self.width - 1, max_x))
        min_y = max(0, min(self.height - 1, min_y))
        max_y = max(0, min(self.height - 1, max_y))

        x0, x1 = sorted((min_x, max_x))
        y0, y1 = sorted((min_y, max_y))
        grid[y0:y1 + 1, x0:x1 + 1] = 1

    def _paint_cylinder(self, grid: np.ndarray, cx: float, cy: float, radius: float) -> None:
        gx, gy = self.world_to_grid(cx, cy)
        rr = max(1, int(math.ceil(radius / self.resolution)))
        y_idxs, x_idxs = np.ogrid[-rr:rr + 1, -rr:rr + 1]
        mask = x_idxs * x_idxs + y_idxs * y_idxs <= rr * rr

        y_min = max(0, gy - rr)
        y_max = min(self.height, gy + rr + 1)
        x_min = max(0, gx - rr)
        x_max = min(self.width, gx + rr + 1)

        sub = grid[y_min:y_max, x_min:x_max]
        m = mask[(y_min - (gy - rr)):(y_max - (gy - rr)), (x_min - (gx - rr)):(x_max - (gx - rr))]
        sub[m] = 1

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        map_msg = OccupancyGrid()
        map_msg.header.stamp = now
        map_msg.header.frame_id = self.frame_id
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = self._occupancy.flatten().tolist()
        self.grid_pub.publish(map_msg)

        markers = MarkerArray()
        for idx, obs in enumerate(DEFAULT_OBSTACLES):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = 'obstacles'
            marker.id = idx
            marker.action = Marker.ADD
            marker.pose.position.x = obs['x']
            marker.pose.position.y = obs['y']
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0
            marker.color.r = 0.85
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 0.75

            if obs['type'] == 'box':
                marker.type = Marker.CUBE
                marker.scale.x = obs['sx']
                marker.scale.y = obs['sy']
                marker.scale.z = 0.5
            else:
                marker.type = Marker.CYLINDER
                marker.scale.x = obs['r'] * 2.0
                marker.scale.y = obs['r'] * 2.0
                marker.scale.z = 0.5

            markers.markers.append(marker)

        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
