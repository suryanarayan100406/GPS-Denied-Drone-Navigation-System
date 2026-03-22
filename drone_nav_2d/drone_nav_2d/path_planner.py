import math
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import Bool

from .prm_planner import PRMPlanner2D
from .informed_rrt_star import InformedRRTStar2D
from .dstar_lite import DStarLite2D


GridCell = Tuple[int, int]


@dataclass
class PlannerState:
    grid: Optional[np.ndarray] = None
    resolution: float = 0.1
    width: int = 100
    height: int = 100
    origin_x: float = -5.0
    origin_y: float = -5.0
    last_grid: Optional[np.ndarray] = None


class PathPlanner(Node):
    """2D Path Planner using PRM + Informed RRT* + D* Lite.
    
    Uses:
    - PRM for roadmap building (speed)
    - Informed RRT* for high-quality path planning
    - D* Lite for dynamic replanning with obstacles
    """

    def __init__(self) -> None:
        super().__init__('path_planner')

        # PRM Parameters
        self.declare_parameter('prm_num_samples', 400)
        self.declare_parameter('prm_connection_radius', 2.0)
        self.declare_parameter('prm_max_connections', 15)
        
        # Informed RRT* Parameters
        self.declare_parameter('irrt_max_iterations', 3000)
        self.declare_parameter('irrt_step_size_m', 0.4)
        self.declare_parameter('irrt_goal_sample_rate', 0.12)
        self.declare_parameter('irrt_rewire_radius_factor', 30.0)
        
        # D* Lite Parameters  
        self.declare_parameter('dstar_enabled', True)
        
        # General Parameters
        self.declare_parameter('map_resolution', 0.1)
        self.declare_parameter('map_width_m', 10.0)
        self.declare_parameter('map_height_m', 10.0)
        self.declare_parameter('inflation_radius_m', 0.4)
        self.declare_parameter('drone_radius_m', 0.2)
        self.declare_parameter('replan_rate_hz', 1.0)
        self.declare_parameter('start_xy', [-4.0, 0.0])
        self.declare_parameter('goal_xy', [4.0, 0.0])
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('replan_min_start_shift_m', 0.35)

        # Get parameters
        self.prm_num_samples = int(self.get_parameter('prm_num_samples').value)
        self.prm_connection_radius = float(self.get_parameter('prm_connection_radius').value)
        self.prm_max_connections = int(self.get_parameter('prm_max_connections').value)
        
        self.irrt_max_iterations = int(self.get_parameter('irrt_max_iterations').value)
        self.irrt_step_size_m = float(self.get_parameter('irrt_step_size_m').value)
        self.irrt_goal_sample_rate = float(self.get_parameter('irrt_goal_sample_rate').value)
        self.irrt_rewire_radius_factor = float(self.get_parameter('irrt_rewire_radius_factor').value)
        
        self.dstar_enabled = bool(self.get_parameter('dstar_enabled').value)
        
        self.start_xy = tuple(self.get_parameter('start_xy').value)
        self.goal_xy = tuple(self.get_parameter('goal_xy').value)
        replan_rate_hz = float(self.get_parameter('replan_rate_hz').value)
        self.replan_min_start_shift_m = float(self.get_parameter('replan_min_start_shift_m').value)

        random_seed = int(self.get_parameter('random_seed').value)
        random.seed(random_seed)
        np.random.seed(random_seed)

        self.state = PlannerState()
        self.current_pose: Optional[PoseStamped] = None
        self.replan_requested = True
        self.mission_completed = False
        self.last_path: List[Tuple[float, float]] = []
        self.last_start_cell: Optional[GridCell] = None
        self.last_goal_cell: Optional[GridCell] = None
        self.last_start_world: Optional[Tuple[float, float]] = None
        self.map_initialized = False
        
        # Algorithm instances
        self.prm: Optional[PRMPlanner2D] = None
        self.irrt_star = InformedRRTStar2D(
            max_iterations=self.irrt_max_iterations,
            step_size=self.irrt_step_size_m,
            goal_sample_rate=self.irrt_goal_sample_rate,
            rewire_radius_factor=self.irrt_rewire_radius_factor,
            collision_checker=None,  # Set after grid is available
        )
        self.dstar: Optional[DStarLite2D] = None
        self.roadmap_built = False

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.replan_event_pub = self.create_publisher(Bool, '/replan_event', 10)

        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(PoseStamped, '/drone_pose', self._on_pose, 10)
        self.create_subscription(Bool, '/replan_request', self._on_replan_request, 10)
        self.create_subscription(Bool, '/mission_complete', self._on_mission_complete, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal_pose, 10)
        self.create_subscription(PointStamped, '/clicked_point', self._on_clicked_point, 10)

        self.timer = self.create_timer(max(0.1, 1.0 / replan_rate_hz), self._plan_if_needed)
        self.get_logger().info('Path planner (PRM + Informed RRT* + D* Lite) initialized.')

    def _on_map(self, msg: OccupancyGrid) -> None:
        data = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        map_changed = (
            self.state.grid is None
            or self.state.grid.shape != data.shape
            or not np.array_equal(self.state.grid, data)
        )

        self.state.last_grid = self.state.grid
        self.state.grid = data
        self.state.resolution = msg.info.resolution
        self.state.width = msg.info.width
        self.state.height = msg.info.height
        self.state.origin_x = msg.info.origin.position.x
        self.state.origin_y = msg.info.origin.position.y

        if not self.map_initialized:
            self.replan_requested = True
            self.roadmap_built = False
            self.map_initialized = True
            return

        if map_changed and not self.mission_completed:
            # Environment changed - trigger D* Lite replanning
            self.replan_requested = True
            self._detect_obstacle_changes()
            if not self.roadmap_built:
                self.roadmap_built = False  # Rebuild PRM roadmap
    
    def _detect_obstacle_changes(self) -> None:
        """Detect which obstacles changed and update D* Lite."""
        if self.state.last_grid is None or self.dstar is None:
            return
        
        # Compare grids to find changed cells
        diff = self.state.grid != self.state.last_grid
        changed_cells = np.where(diff)
        
        if len(changed_cells[0]) == 0:
            return
        
        # Update D* Lite with changed obstacles
        for gy, gx in zip(changed_cells[0], changed_cells[1]):
            world_x = self.state.origin_x + (gx + 0.5) * self.state.resolution
            world_y = self.state.origin_y + (gy + 0.5) * self.state.resolution
            is_blocked = self.state.grid[gy, gx] >= 50
            # D* Lite will handle the update in next planning cycle

    def _on_pose(self, msg: PoseStamped) -> None:
        self.current_pose = msg

    def _on_replan_request(self, msg: Bool) -> None:
        if msg.data:
            self.replan_requested = True

    def _on_mission_complete(self, msg: Bool) -> None:
        if msg.data:
            self.mission_completed = True

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        self.goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.replan_requested = True
        self.mission_completed = False
        self.get_logger().info(
            f'New user goal received on /goal_pose: ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f})'
        )

    def _on_clicked_point(self, msg: PointStamped) -> None:
        self.goal_xy = (msg.point.x, msg.point.y)
        self.replan_requested = True
        self.mission_completed = False
        self.get_logger().info(
            f'New user goal received on /clicked_point: ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f})'
        )

    def _path_changed(self, path_world: List[Tuple[float, float]]) -> bool:
        if len(path_world) != len(self.last_path):
            return True
        if not path_world:
            return False
        if not self.last_path:
            return True
        first_delta = math.hypot(path_world[0][0] - self.last_path[0][0], path_world[0][1] - self.last_path[0][1])
        last_delta = math.hypot(path_world[-1][0] - self.last_path[-1][0], path_world[-1][1] - self.last_path[-1][1])
        return first_delta > self.state.resolution or last_delta > self.state.resolution

    def _plan_if_needed(self) -> None:
        if self.state.grid is None:
            return
        if self.mission_completed and not self.replan_requested:
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

        start_shift = float('inf')
        if self.last_start_world is not None:
            start_shift = math.hypot(
                start_world[0] - self.last_start_world[0],
                start_world[1] - self.last_start_world[1],
            )

        # Skip replanning if not needed
        if (
            not self.replan_requested
            and self.last_goal_cell == goal
            and start_shift < self.replan_min_start_shift_m
        ):
            return

        if not self._is_free(start) or not self._is_free(goal):
            self.get_logger().warn('Start or goal in obstacle. Waiting for valid state.')
            return

        # Build PRM roadmap if needed
        if not self.roadmap_built:
            self._build_prm_roadmap()
        
        # Update collision checker for IRRT*
        self.irrt_star.collision_checker = self._is_free_world
        
        # Plan path using Informed RRT*
        bounds = (
            self.state.origin_x,
            self.state.origin_x + self.state.width * self.state.resolution,
            self.state.origin_y,
            self.state.origin_y + self.state.height * self.state.resolution,
        )
        
        path_world = self.irrt_star.plan(start_world, self.goal_xy, bounds)
        
        # Fallback to D* Lite if Informed RRT* fails
        if not path_world and self.dstar_enabled:
            self.get_logger().warn('Informed RRT* found no path; using D* Lite fallback.')
            if self.dstar is None:
                self.dstar = DStarLite2D(
                    resolution=self.state.resolution,
                    collision_checker=self._is_free_world,
                )
            path_world = self.dstar.plan(
                start_world, self.goal_xy,
                self.state.width, self.state.height,
                self.state.origin_x, self.state.origin_y,
            )
        
        if not path_world:
            self.get_logger().error('No valid path found.')
            return

        should_publish = self.replan_requested or self._path_changed(path_world)

        self.last_start_cell = start
        self.last_goal_cell = goal
        self.last_start_world = start_world

        if not should_publish:
            return

        self.last_path = path_world
        self.publish_path(self.last_path)

        event_msg = Bool()
        event_msg.data = self.replan_requested
        self.replan_event_pub.publish(event_msg)

        self.replan_requested = False
    
    def _build_prm_roadmap(self) -> None:
        """Build PRM roadmap for fast planning."""
        self.get_logger().info(f'Building PRM roadmap ({self.prm_num_samples} samples)...')
        
        self.prm = PRMPlanner2D(
            num_samples=self.prm_num_samples,
            connection_radius=self.prm_connection_radius,
            max_connections=self.prm_max_connections,
            collision_checker=self._is_free_world,
        )
        
        bounds = (
            self.state.origin_x,
            self.state.origin_x + self.state.width * self.state.resolution,
            self.state.origin_y,
            self.state.origin_y + self.state.height * self.state.resolution,
        )
        
        if self.prm.build_roadmap(bounds):
            self.roadmap_built = True
            self.get_logger().info(f'PRM roadmap built with {len(self.prm.nodes)} nodes.')
        else:
            self.get_logger().warn('Failed to build PRM roadmap.')
            self.roadmap_built = False

    def _is_free(self, cell: GridCell) -> bool:
        """Check if grid cell is collision-free."""
        gx, gy = cell
        if gx < 0 or gy < 0 or gx >= self.state.width or gy >= self.state.height:
            return False
        return self.state.grid[gy, gx] < 50
    
    def _is_free_world(self, pos: Tuple[float, float]) -> bool:
        """Check if world position is collision-free."""
        if self.state.grid is None:
            return True
        
        cell = self.world_to_grid(pos[0], pos[1])
        return self._is_free(cell)

    def _path_changed(self, path_world: List[Tuple[float, float]]) -> bool:
        if len(path_world) != len(self.last_path):
            return True
        if not path_world:
            return False
        if not self.last_path:
            return True
        first_delta = math.hypot(path_world[0][0] - self.last_path[0][0], path_world[0][1] - self.last_path[0][1])
        last_delta = math.hypot(path_world[-1][0] - self.last_path[-1][0], path_world[-1][1] - self.last_path[-1][1])
        return first_delta > self.state.resolution or last_delta > self.state.resolution

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
