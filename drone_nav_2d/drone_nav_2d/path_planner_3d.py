import math
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool

from .voxel_grid import VoxelGrid
from .prm_planner import PRMPlanner3D
from .informed_rrt_star import InformedRRTStar3D
from .dstar_lite import DStarLite3D


VoxelCell = Tuple[int, int, int]


@dataclass
class Planner3DState:
    """State for 3D path planner."""
    voxel_grid: Optional[VoxelGrid] = None
    drone_radius_m: float = 0.2


class PathPlanner3D(Node):
    """3D Path Planner using PRM + Informed RRT* + D* Lite.
    
    Uses:
    - PRM for roadmap building (speed)
    - Informed RRT* for high-quality path planning
    - D* Lite for dynamic replanning with moving obstacles
    
    Operates on voxel-based 3D grid with 26-connectivity support.
    """

    def __init__(self) -> None:
        super().__init__('path_planner_3d')

        # PRM Parameters
        self.declare_parameter('prm_num_samples', 600)
        self.declare_parameter('prm_connection_radius', 2.5)
        self.declare_parameter('prm_max_connections', 20)
        
        # Informed RRT* Parameters
        self.declare_parameter('irrt_max_iterations', 4000)
        self.declare_parameter('irrt_step_size_m', 0.5)
        self.declare_parameter('irrt_goal_sample_rate', 0.12)
        self.declare_parameter('irrt_rewire_radius_factor', 35.0)
        
        # D* Lite Parameters
        self.declare_parameter('dstar_enabled', True)
        
        # Voxel Parameters
        self.declare_parameter('voxel_resolution', 0.2)
        self.declare_parameter('world_width_m', 10.0)
        self.declare_parameter('world_height_m', 8.0)
        self.declare_parameter('world_depth_m', 10.0)
        self.declare_parameter('inflation_radius_m', 0.4)
        self.declare_parameter('drone_radius_m', 0.2)
        self.declare_parameter('replan_rate_hz', 1.0)
        self.declare_parameter('start_xyz', [-4.0, 0.0, 1.0])
        self.declare_parameter('goal_xyz', [4.0, 0.0, 2.0])

        # Get parameters
        self.prm_num_samples = int(self.get_parameter('prm_num_samples').value)
        self.prm_connection_radius = float(self.get_parameter('prm_connection_radius').value)
        self.prm_max_connections = int(self.get_parameter('prm_max_connections').value)
        
        self.irrt_max_iterations = int(self.get_parameter('irrt_max_iterations').value)
        self.irrt_step_size_m = float(self.get_parameter('irrt_step_size_m').value)
        self.irrt_goal_sample_rate = float(self.get_parameter('irrt_goal_sample_rate').value)
        self.irrt_rewire_radius_factor = float(self.get_parameter('irrt_rewire_radius_factor').value)
        
        self.dstar_enabled = bool(self.get_parameter('dstar_enabled').value)
        
        # Voxel grid setup
        voxel_res = float(self.get_parameter('voxel_resolution').value)
        world_width = float(self.get_parameter('world_width_m').value)
        world_height = float(self.get_parameter('world_height_m').value)
        world_depth = float(self.get_parameter('world_depth_m').value)
        inflation_rad = float(self.get_parameter('inflation_radius_m').value)
        drone_rad = float(self.get_parameter('drone_radius_m').value)
        replan_rate = float(self.get_parameter('replan_rate_hz').value)
        self.start_xyz = tuple(self.get_parameter('start_xyz').value)
        self.goal_xyz = tuple(self.get_parameter('goal_xyz').value)

        # Initialize voxel grid
        self.state = Planner3DState()
        self.state.voxel_grid = VoxelGrid(
            width_m=world_width,
            height_m=world_height,
            depth_m=world_depth,
            resolution=voxel_res,
        )
        self.state.voxel_grid.set_inflation_radius(inflation_rad)
        self.state.drone_radius_m = drone_rad

        # Planning state
        self.current_pose: Optional[PoseStamped] = None
        self.replan_requested = True
        self.mission_completed = False
        self.last_path: List[Tuple[float, float, float]] = []
        self.last_start_cell: Optional[VoxelCell] = None
        self.last_goal_cell: Optional[VoxelCell] = None
        self.map_initialized = False
        
        # Algorithm instances
        self.prm: Optional[PRMPlanner3D] = None
        self.irrt_star = InformedRRTStar3D(
            max_iterations=self.irrt_max_iterations,
            step_size=self.irrt_step_size_m,
            goal_sample_rate=self.irrt_goal_sample_rate,
            rewire_radius_factor=self.irrt_rewire_radius_factor,
            collision_checker=None,
        )
        self.dstar: Optional[DStarLite3D] = None
        self.roadmap_built = False

        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path_3d', 10)
        self.replan_event_pub = self.create_publisher(Bool, '/replan_event_3d', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/drone_pose_3d', self._on_pose, 10)
        self.create_subscription(Bool, '/replan_request_3d', self._on_replan_request, 10)
        self.create_subscription(Bool, '/mission_complete_3d', self._on_mission_complete, 10)
        self.create_subscription(PoseStamped, '/goal_pose_3d', self._on_goal_pose, 10)
        self.create_subscription(PointStamped, '/clicked_point_3d', self._on_clicked_point, 10)

        # Timer for planning
        self.timer = self.create_timer(max(0.1, 1.0 / replan_rate), self._plan_if_needed)

        self.get_logger().info('3D Path planner (PRM + Informed RRT* + D* Lite) initialized.')

    def _on_pose(self, msg: PoseStamped) -> None:
        """Update current drone pose."""
        self.current_pose = msg

    def _on_replan_request(self, msg: Bool) -> None:
        """Request replanning (e.g., obstacle detected)."""
        if msg.data:
            self.replan_requested = True

    def _on_mission_complete(self, msg: Bool) -> None:
        """Mark mission as complete."""
        if msg.data:
            self.mission_completed = True

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        """Receive new goal pose."""
        self.goal_xyz = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        self.replan_requested = True
        self.mission_completed = False
        self.get_logger().info(f'New goal: ({self.goal_xyz[0]:.2f}, {self.goal_xyz[1]:.2f}, {self.goal_xyz[2]:.2f})')

    def _on_clicked_point(self, msg: PointStamped) -> None:
        """Receive goal from RViz click."""
        self.goal_xyz = (msg.point.x, msg.point.y, msg.point.z)
        self.replan_requested = True
        self.mission_completed = False
        self.get_logger().info(f'New goal (RViz): ({self.goal_xyz[0]:.2f}, {self.goal_xyz[1]:.2f}, {self.goal_xyz[2]:.2f})')

    def _plan_if_needed(self) -> None:
        """Main planning loop."""
        if self.state.voxel_grid is None:
            return

        if self.mission_completed and not self.replan_requested:
            return

        # Get current position
        if self.current_pose is not None:
            start_xyz = (
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z,
            )
        else:
            start_xyz = self.start_xyz

        # Convert to voxel coordinates
        start_cell = self.state.voxel_grid.world_to_voxel(*start_xyz)
        goal_cell = self.state.voxel_grid.world_to_voxel(*self.goal_xyz)

        # Check if replanning needed
        if (
            not self.replan_requested
            and self.last_goal_cell == goal_cell
            and self.last_start_cell is not None
            and self.last_start_cell == start_cell
        ):
            return

        # Validate start/goal are free
        if not self.state.voxel_grid.is_free(*start_xyz, use_inflation=True):
            self.get_logger().warn('Start position blocked.')
            return

        if not self.state.voxel_grid.is_free(*self.goal_xyz, use_inflation=True):
            self.get_logger().warn('Goal position blocked.')
            return

        # Build PRM if needed
        if not self.roadmap_built:
            self._build_prm_roadmap()
        
        # Setup collision checker
        self.irrt_star.collision_checker = self._is_free_world
        
        # Plan using Informed RRT*
        bounds = (
            -self.state.voxel_grid.width_m / 2,
            self.state.voxel_grid.width_m / 2,
            -self.state.voxel_grid.depth_m / 2,
            self.state.voxel_grid.depth_m / 2,
            0,
            self.state.voxel_grid.height_m,
        )
        
        path_world = self.irrt_star.plan(start_xyz, self.goal_xyz, bounds)
        
        # Fallback to D* Lite
        if not path_world and self.dstar_enabled:
            self.get_logger().warn('Informed RRT* found no path; using D* Lite fallback.')
            if self.dstar is None:
                self.dstar = DStarLite3D(
                    resolution=self.state.voxel_grid.resolution,
                    collision_checker=self._is_free_world,
                )
            path_world = self.dstar.plan(
                start_xyz, self.goal_xyz,
                self.state.voxel_grid.width,
                self.state.voxel_grid.depth,
                self.state.voxel_grid.height,
            )
        
        if not path_world:
            self.get_logger().error('No valid path found.')
            return

        # Publish path
        self.publish_path(path_world)
        self.last_path = path_world
        self.last_start_cell = start_cell
        self.last_goal_cell = goal_cell

        # Publish replan event
        event_msg = Bool()
        event_msg.data = self.replan_requested
        self.replan_event_pub.publish(event_msg)

        self.replan_requested = False
    
    def _build_prm_roadmap(self) -> None:
        """Build 3D PRM roadmap for fast planning."""
        self.get_logger().info(f'Building 3D PRM roadmap ({self.prm_num_samples} samples)...')
        
        self.prm = PRMPlanner3D(
            num_samples=self.prm_num_samples,
            connection_radius=self.prm_connection_radius,
            max_connections=self.prm_max_connections,
            collision_checker=self._is_free_world,
        )
        
        bounds = (
            -self.state.voxel_grid.width_m / 2,
            self.state.voxel_grid.width_m / 2,
            -self.state.voxel_grid.depth_m / 2,
            self.state.voxel_grid.depth_m / 2,
            0,
            self.state.voxel_grid.height_m,
        )
        
        if self.prm.build_roadmap(bounds):
            self.roadmap_built = True
            self.get_logger().info(f'3D PRM roadmap built with {len(self.prm.nodes)} nodes.')
        else:
            self.get_logger().warn('Failed to build 3D PRM roadmap.')
            self.roadmap_built = False
    
    def _is_free_world(self, pos: Tuple[float, float, float]) -> bool:
        """Check if 3D world position is collision-free."""
        if self.state.voxel_grid is None:
            return True
        return self.state.voxel_grid.is_free(pos[0], pos[1], pos[2], use_inflation=True)

    def publish_path(self, points: List[Tuple[float, float, float]]) -> None:
        """Publish path as ROS message."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for x, y, z in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self.get_logger().info(f'Published 3D path with {len(msg.poses)} waypoints.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathPlanner3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
