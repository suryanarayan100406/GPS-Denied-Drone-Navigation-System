import heapq
import math
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import Bool


GridCell = Tuple[int, int]


@dataclass
class PlannerState:
    grid: Optional[np.ndarray] = None
    resolution: float = 0.1
    width: int = 100
    height: int = 100
    origin_x: float = -5.0
    origin_y: float = -5.0


class PathPlanner(Node):
    def __init__(self) -> None:
        super().__init__('path_planner')

        self.declare_parameter('map_resolution', 0.1)
        self.declare_parameter('map_width_m', 10.0)
        self.declare_parameter('map_height_m', 10.0)
        self.declare_parameter('inflation_radius_m', 0.4)
        self.declare_parameter('drone_radius_m', 0.2)
        self.declare_parameter('heuristic_weight_manhattan', 0.7)
        self.declare_parameter('heuristic_weight_euclidean', 0.3)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('replan_rate_hz', 1.0)
        self.declare_parameter('start_xy', [-4.0, 0.0])
        self.declare_parameter('goal_xy', [4.0, 0.0])
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('rrt_max_iterations', 2500)
        self.declare_parameter('rrt_step_size_m', 0.35)
        self.declare_parameter('rrt_goal_sample_rate', 0.2)

        self.w_manhattan = self.get_parameter('heuristic_weight_manhattan').value
        self.w_euclidean = self.get_parameter('heuristic_weight_euclidean').value
        self.allow_diagonal = bool(self.get_parameter('allow_diagonal').value)
        self.start_xy = tuple(self.get_parameter('start_xy').value)
        self.goal_xy = tuple(self.get_parameter('goal_xy').value)
        replan_rate_hz = float(self.get_parameter('replan_rate_hz').value)

        random_seed = int(self.get_parameter('random_seed').value)
        random.seed(random_seed)
        np.random.seed(random_seed)

        self.state = PlannerState()
        self.current_pose: Optional[PoseStamped] = None
        self.replan_requested = True
        self.last_path: List[Tuple[float, float]] = []

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.replan_event_pub = self.create_publisher(Bool, '/replan_event', 10)

        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(PoseStamped, '/drone_pose', self._on_pose, 10)
        self.create_subscription(Bool, '/replan_request', self._on_replan_request, 10)

        self.timer = self.create_timer(max(0.1, 1.0 / replan_rate_hz), self._plan_if_needed)
        self.get_logger().info('Path planner initialized.')

    def _on_map(self, msg: OccupancyGrid) -> None:
        data = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        self.state.grid = data
        self.state.resolution = msg.info.resolution
        self.state.width = msg.info.width
        self.state.height = msg.info.height
        self.state.origin_x = msg.info.origin.position.x
        self.state.origin_y = msg.info.origin.position.y
        self.replan_requested = True

    def _on_pose(self, msg: PoseStamped) -> None:
        self.current_pose = msg

    def _on_replan_request(self, msg: Bool) -> None:
        if msg.data:
            self.replan_requested = True

    def _plan_if_needed(self) -> None:
        if self.state.grid is None:
            return
        if self.current_pose is None and not self.replan_requested:
            return

        start_world = self.start_xy
        if self.current_pose is not None:
            start_world = (
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
            )

        start = self.world_to_grid(*start_world)
        goal = self.world_to_grid(*self.goal_xy)

        if not self._is_free(start) or not self._is_free(goal):
            self.get_logger().warn('Start or goal lies in obstacle after inflation. Waiting for valid state.')
            return

        path_grid = self.a_star(start, goal)
        used_rrt = False

        if not path_grid:
            self.get_logger().warn('A* found no path; attempting RRT fallback.')
            path_grid = self.rrt_plan(start, goal)
            used_rrt = True

        if not path_grid:
            self.get_logger().error('No valid path found by A* or RRT.')
            return

        self.last_path = [self.grid_to_world(c[0], c[1]) for c in path_grid]
        self.publish_path(self.last_path)

        event_msg = Bool()
        event_msg.data = bool(self.replan_requested or used_rrt)
        self.replan_event_pub.publish(event_msg)

        self.replan_requested = False

    def world_to_grid(self, x: float, y: float) -> GridCell:
        gx = int((x - self.state.origin_x) / self.state.resolution)
        gy = int((y - self.state.origin_y) / self.state.resolution)
        gx = max(0, min(self.state.width - 1, gx))
        gy = max(0, min(self.state.height - 1, gy))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = self.state.origin_x + (gx + 0.5) * self.state.resolution
        y = self.state.origin_y + (gy + 0.5) * self.state.resolution
        return x, y

    def _is_free(self, cell: GridCell) -> bool:
        gx, gy = cell
        if gx < 0 or gy < 0 or gx >= self.state.width or gy >= self.state.height:
            return False
        return self.state.grid[gy, gx] < 50

    def _neighbors(self, cell: GridCell) -> List[GridCell]:
        x, y = cell
        if self.allow_diagonal:
            deltas = [
                (-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (1, -1), (-1, 1), (1, 1),
            ]
        else:
            deltas = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        result = []
        for dx, dy in deltas:
            nb = (x + dx, y + dy)
            if self._is_free(nb):
                result.append(nb)
        return result

    def _heuristic(self, a: GridCell, b: GridCell) -> float:
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        manhattan = dx + dy
        euclidean = math.sqrt(dx * dx + dy * dy)
        return self.w_manhattan * manhattan + self.w_euclidean * euclidean

    def a_star(self, start: GridCell, goal: GridCell) -> List[GridCell]:
        open_heap: List[Tuple[float, GridCell]] = []
        heapq.heappush(open_heap, (0.0, start))

        came_from: Dict[GridCell, GridCell] = {}
        g_score: Dict[GridCell, float] = {start: 0.0}
        f_score: Dict[GridCell, float] = {start: self._heuristic(start, goal)}
        open_set = {start}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current not in open_set:
                continue
            open_set.remove(current)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for nb in self._neighbors(current):
                step_cost = math.sqrt(2.0) if (nb[0] != current[0] and nb[1] != current[1]) else 1.0
                tentative = g_score[current] + step_cost
                if tentative < g_score.get(nb, float('inf')):
                    came_from[nb] = current
                    g_score[nb] = tentative
                    f_score[nb] = tentative + self._heuristic(nb, goal)
                    if nb not in open_set:
                        open_set.add(nb)
                        heapq.heappush(open_heap, (f_score[nb], nb))

        return []

    def _reconstruct_path(self, came_from: Dict[GridCell, GridCell], current: GridCell) -> List[GridCell]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def rrt_plan(self, start: GridCell, goal: GridCell) -> List[GridCell]:
        max_iter = int(self.get_parameter('rrt_max_iterations').value)
        step_size_m = float(self.get_parameter('rrt_step_size_m').value)
        goal_sample_rate = float(self.get_parameter('rrt_goal_sample_rate').value)
        step_cells = max(1, int(step_size_m / self.state.resolution))

        tree: List[GridCell] = [start]
        parent: Dict[GridCell, Optional[GridCell]] = {start: None}

        for _ in range(max_iter):
            if random.random() < goal_sample_rate:
                sample = goal
            else:
                sample = (
                    random.randint(0, self.state.width - 1),
                    random.randint(0, self.state.height - 1),
                )

            nearest = min(tree, key=lambda n: (n[0] - sample[0]) ** 2 + (n[1] - sample[1]) ** 2)
            new_node = self._steer(nearest, sample, step_cells)

            if not self._is_free(new_node):
                continue
            if not self._line_free(nearest, new_node):
                continue
            if new_node in parent:
                continue

            tree.append(new_node)
            parent[new_node] = nearest

            if self._distance_cells(new_node, goal) <= step_cells and self._line_free(new_node, goal):
                parent[goal] = new_node
                return self._rrt_reconstruct(parent, goal)

        return []

    def _steer(self, from_node: GridCell, to_node: GridCell, step_cells: int) -> GridCell:
        dx = to_node[0] - from_node[0]
        dy = to_node[1] - from_node[1]
        dist = math.sqrt(dx * dx + dy * dy)
        if dist <= step_cells:
            return to_node
        nx = int(round(from_node[0] + dx / dist * step_cells))
        ny = int(round(from_node[1] + dy / dist * step_cells))
        nx = max(0, min(self.state.width - 1, nx))
        ny = max(0, min(self.state.height - 1, ny))
        return nx, ny

    def _line_free(self, a: GridCell, b: GridCell) -> bool:
        x0, y0 = a
        x1, y1 = b
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        for i in range(steps + 1):
            t = i / steps
            x = int(round(x0 + (x1 - x0) * t))
            y = int(round(y0 + (y1 - y0) * t))
            if not self._is_free((x, y)):
                return False
        return True

    def _distance_cells(self, a: GridCell, b: GridCell) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def _rrt_reconstruct(self, parent: Dict[GridCell, Optional[GridCell]], goal: GridCell) -> List[GridCell]:
        path = [goal]
        current = goal
        while parent[current] is not None:
            current = parent[current]
            path.append(current)
        path.reverse()
        return path

    def publish_path(self, points: List[Tuple[float, float]]) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.5
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self.get_logger().info(f'Published planned path with {len(msg.poses)} waypoints.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
