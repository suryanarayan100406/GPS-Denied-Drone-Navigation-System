"""D* Lite Path Planning Algorithm.

D* Lite is an incremental heuristic search algorithm that efficiently
handles replanning when the environment changes or new obstacles appear.

Perfect for dynamic environments with moving obstacles.

Key features:
- Incremental replanning
- Handles dynamic obstacles efficiently
- Maintains consistency in dynamic environments
"""

import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

import numpy as np


GridCell = Tuple[int, int]
VoxelCell = Tuple[int, int, int]


class DStarLite2D:
    """D* Lite path planner for dynamic 2D environments."""

    def __init__(
        self,
        resolution: float = 0.1,
        collision_checker=None,
    ):
        """Initialize D* Lite planner.
        
        Args:
            resolution: Grid resolution
            collision_checker: Function to check collision
        """
        self.resolution = resolution
        self.collision_checker = collision_checker
        
        # Planning state
        self.g_values: Dict[GridCell, float] = {}
        self.rhs_values: Dict[GridCell, float] = {}
        self.open_set: List[Tuple[Tuple[float, float], GridCell]] = []
        
        self.start: Optional[GridCell] = None
        self.goal: Optional[GridCell] = None
        self.last_start: Optional[GridCell] = None
        
        self.km = 0.0
        self.changed_edges: Set[GridCell] = set()
    
    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        width: int,
        height: int,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
    ) -> List[Tuple[float, float]]:
        """Plan path using D* Lite.
        
        Args:
            start: Start position (world coords)
            goal: Goal position (world coords)
            width: Grid width
            height: Grid height
            origin_x: Grid origin X
            origin_y: Grid origin Y
            
        Returns:
            Path from start to goal
        """
        # Convert to grid
        start_cell = self._world_to_grid(start, origin_x, origin_y)
        goal_cell = self._world_to_grid(goal, origin_x, origin_y)
        
        # Initialize or update for new goal
        if self.goal != goal_cell:
            self.start = start_cell
            self.goal = goal_cell
            self.last_start = start_cell
            
            self.g_values = {}
            self.rhs_values = {}
            self.open_set = []
            self.km = 0.0
            self.changed_edges = set()
            
            # Initialize
            self.rhs_values[goal_cell] = 0.0
            key = self._calculate_key(goal_cell)
            heapq.heappush(self.open_set, (key, goal_cell))
        else:
            # Goal same, but start might have moved
            if start_cell != self.last_start:
                self.km += self._heuristic(self.last_start, start_cell)
                self.last_start = start_cell
        
        self.start = start_cell
        
        # Main D* Lite loop
        while True:
            key_start = self._calculate_key(start_cell)
            
            if self.open_set:
                key_top = self.open_set[0][0]
            else:
                key_top = (float('inf'), float('inf'))
            
            # Process expansions
            if key_top <= key_start and (start_cell in self.rhs_values):
                if self.g_values.get(start_cell, float('inf')) == self.rhs_values.get(start_cell, float('inf')):
                    break  # Path found
                else:
                    self.g_values[start_cell] = self.rhs_values[start_cell]
                    self._remove_from_open(start_cell)
            elif key_top < key_start:
                self._expand_node(start_cell, goal_cell)
            else:
                break
        
        # Reconstruct path
        return self._reconstruct_path(start_cell, goal_cell, origin_x, origin_y)
    
    def update_edge(
        self,
        obstacle: Tuple[float, float],
        is_blocked: bool,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        width: int,
        height: int,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
    ) -> List[Tuple[float, float]]:
        """Update path when environment changes.
        
        Args:
            obstacle: Obstacle position
            is_blocked: True if blocked, False if unblocked
            start: Current start
            goal: Current goal
            width: Grid width
            height: Grid height
            origin_x: Grid origin X
            origin_y: Grid origin Y
            
        Returns:
            Updated path
        """
        obs_cell = self._world_to_grid(obstacle, origin_x, origin_y)
        
        # Mark affected edges for update
        for neighbor in self._get_neighbors(obs_cell, width, height):
            if is_blocked:
                self.changed_edges.add((obs_cell, neighbor))
                self.changed_edges.add((neighbor, obs_cell))
            else:
                self.changed_edges.discard((obs_cell, neighbor))
                self.changed_edges.discard((neighbor, obs_cell))
        
        # Update affected cells
        self._update_affected_cells(obs_cell, width, height)
        
        # Replan
        return self.plan(start, goal, width, height, origin_x, origin_y)
    
    def _expand_node(self, start: GridCell, goal: GridCell) -> None:
        """Expand a node in D* Lite."""
        key, u = heapq.heappop(self.open_set)
        
        if self.g_values.get(u, float('inf')) > self.rhs_values.get(u, float('inf')):
            self.g_values[u] = self.rhs_values[u]
        else:
            self.g_values[u] = float('inf')
        
        # Update neighbors
        for v in self._get_neighbors(u, 100, 100):
            if v != goal:
                self.rhs_values[v] = min(
                    self.rhs_values.get(v, float('inf')),
                    self.g_values.get(u, float('inf')) + self._heuristic(u, v),
                )
            
            if v != start:
                key_v = self._calculate_key(v)
                if (key_v, v) in self.open_set:
                    self._remove_from_open(v)
                
                if self.g_values.get(v, float('inf')) != self.rhs_values.get(v, float('inf')):
                    heapq.heappush(self.open_set, (key_v, v))
    
    def _update_affected_cells(self, cell: GridCell, width: int, height: int) -> None:
        """Update cells affected by environment change."""
        for neighbor in self._get_neighbors(cell, width, height):
            if neighbor in self.rhs_values:
                # Recalculate rhs value
                rhs = min(
                    self.rhs_values.get(neighbor, float('inf')),
                    self.g_values.get(neighbor, float('inf')) + 1.0,
                )
                self.rhs_values[neighbor] = rhs
                
                key = self._calculate_key(neighbor)
                if (key, neighbor) in self.open_set:
                    self._remove_from_open(neighbor)
                
                if self.g_values.get(neighbor, float('inf')) != self.rhs_values.get(neighbor, float('inf')):
                    heapq.heappush(self.open_set, (key, neighbor))
    
    def _calculate_key(self, cell: GridCell) -> Tuple[float, float]:
        """Calculate key for open set."""
        g = self.g_values.get(cell, float('inf'))
        rhs = self.rhs_values.get(cell, float('inf'))
        h = self._heuristic(cell, self.start) if self.start else 0.0
        
        return (min(g, rhs) + h + self.km, min(g, rhs))
    
    def _remove_from_open(self, cell: GridCell) -> None:
        """Remove cell from open set."""
        self.open_set = [(k, c) for k, c in self.open_set if c != cell]
        heapq.heapify(self.open_set)
    
    def _reconstruct_path(
        self,
        start: GridCell,
        goal: GridCell,
        origin_x: float,
        origin_y: float,
    ) -> List[Tuple[float, float]]:
        """Reconstruct path from start to goal."""
        path = []
        current = start
        
        while current != goal:
            path.append(self._grid_to_world(current, origin_x, origin_y))
            
            # Find neighbor with minimum g + h
            best_neighbor = None
            best_cost = float('inf')
            
            for neighbor in self._get_neighbors(current, 100, 100):
                cost = self.g_values.get(neighbor, float('inf')) + 1.0
                if cost < best_cost:
                    best_cost = cost
                    best_neighbor = neighbor
            
            if best_neighbor is None:
                break  # No path
            
            current = best_neighbor
            
            if len(path) > 1000:  # Safety check
                break
        
        path.append(self._grid_to_world(goal, origin_x, origin_y))
        return path
    
    @staticmethod
    def _heuristic(a: GridCell, b: GridCell) -> float:
        """Manhattan distance heuristic."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    @staticmethod
    def _get_neighbors(cell: GridCell, width: int, height: int) -> List[GridCell]:
        """Get 4-connected neighbors."""
        x, y = cell
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                neighbors.append((nx, ny))
        return neighbors
    
    def _world_to_grid(
        self,
        pos: Tuple[float, float],
        origin_x: float,
        origin_y: float,
    ) -> GridCell:
        """Convert world to grid coordinates."""
        x = int((pos[0] - origin_x) / self.resolution)
        y = int((pos[1] - origin_y) / self.resolution)
        return (x, y)
    
    def _grid_to_world(
        self,
        cell: GridCell,
        origin_x: float,
        origin_y: float,
    ) -> Tuple[float, float]:
        """Convert grid to world coordinates."""
        x = cell[0] * self.resolution + origin_x
        y = cell[1] * self.resolution + origin_y
        return (x, y)


class DStarLite3D:
    """D* Lite for 3D voxel-based environments."""

    def __init__(
        self,
        resolution: float = 0.2,
        collision_checker=None,
    ):
        """Initialize 3D D* Lite."""
        self.resolution = resolution
        self.collision_checker = collision_checker
        
        self.g_values: Dict[VoxelCell, float] = {}
        self.rhs_values: Dict[VoxelCell, float] = {}
        self.open_set: List[Tuple[Tuple[float, float], VoxelCell]] = []
        
        self.start: Optional[VoxelCell] = None
        self.goal: Optional[VoxelCell] = None
        self.last_start: Optional[VoxelCell] = None
        
        self.km = 0.0
    
    def plan(
        self,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
        width: int,
        depth: int,
        height: int,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        origin_z: float = 0.0,
    ) -> List[Tuple[float, float, float]]:
        """Plan 3D path using D* Lite."""
        start_cell = self._world_to_voxel(start, origin_x, origin_y, origin_z)
        goal_cell = self._world_to_voxel(goal, origin_x, origin_y, origin_z)
        
        if self.goal != goal_cell:
            self.start = start_cell
            self.goal = goal_cell
            self.last_start = start_cell
            
            self.g_values = {}
            self.rhs_values = {}
            self.open_set = []
            self.km = 0.0
            
            self.rhs_values[goal_cell] = 0.0
            key = self._calculate_key(goal_cell)
            heapq.heappush(self.open_set, (key, goal_cell))
        else:
            if start_cell != self.last_start:
                self.km += self._heuristic(self.last_start, start_cell)
                self.last_start = start_cell
        
        self.start = start_cell
        
        # Main loop
        while True:
            key_start = self._calculate_key(start_cell)
            
            if self.open_set:
                key_top = self.open_set[0][0]
            else:
                key_top = (float('inf'), float('inf'))
            
            if key_top <= key_start and (start_cell in self.rhs_values):
                if self.g_values.get(start_cell, float('inf')) == self.rhs_values.get(start_cell, float('inf')):
                    break
                else:
                    self.g_values[start_cell] = self.rhs_values[start_cell]
            elif key_top < key_start:
                self._expand_node_3d(start_cell, goal_cell)
            else:
                break
        
        return self._reconstruct_path_3d(
            start_cell, goal_cell, origin_x, origin_y, origin_z
        )
    
    def _expand_node_3d(self, start: VoxelCell, goal: VoxelCell) -> None:
        """Expand 3D node."""
        key, u = heapq.heappop(self.open_set)
        
        if self.g_values.get(u, float('inf')) > self.rhs_values.get(u, float('inf')):
            self.g_values[u] = self.rhs_values[u]
        else:
            self.g_values[u] = float('inf')
        
        for v in self._get_neighbors_3d(u):
            if v != goal:
                self.rhs_values[v] = min(
                    self.rhs_values.get(v, float('inf')),
                    self.g_values.get(u, float('inf')) + self._distance_3d(u, v),
                )
            
            if v != start:
                key_v = self._calculate_key(v)
                if (key_v, v) in self.open_set:
                    self._remove_from_open(v)
                
                if self.g_values.get(v, float('inf')) != self.rhs_values.get(v, float('inf')):
                    heapq.heappush(self.open_set, (key_v, v))
    
    def _calculate_key(self, cell: VoxelCell) -> Tuple[float, float]:
        """Calculate 3D key."""
        g = self.g_values.get(cell, float('inf'))
        rhs = self.rhs_values.get(cell, float('inf'))
        h = self._heuristic(cell, self.start) if self.start else 0.0
        
        return (min(g, rhs) + h + self.km, min(g, rhs))
    
    def _remove_from_open(self, cell: VoxelCell) -> None:
        """Remove from open set."""
        self.open_set = [(k, c) for k, c in self.open_set if c != cell]
        heapq.heapify(self.open_set)
    
    def _reconstruct_path_3d(
        self,
        start: VoxelCell,
        goal: VoxelCell,
        origin_x: float,
        origin_y: float,
        origin_z: float,
    ) -> List[Tuple[float, float, float]]:
        """Reconstruct 3D path."""
        path = []
        current = start
        
        while current != goal:
            path.append(self._voxel_to_world(current, origin_x, origin_y, origin_z))
            
            best_neighbor = None
            best_cost = float('inf')
            
            for neighbor in self._get_neighbors_3d(current):
                cost = self.g_values.get(neighbor, float('inf')) + self._distance_3d(current, neighbor)
                if cost < best_cost:
                    best_cost = cost
                    best_neighbor = neighbor
            
            if best_neighbor is None:
                break
            
            current = best_neighbor
            
            if len(path) > 2000:
                break
        
        path.append(self._voxel_to_world(goal, origin_x, origin_y, origin_z))
        return path
    
    @staticmethod
    def _heuristic(a: VoxelCell, b: VoxelCell) -> float:
        """3D Manhattan distance."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])
    
    @staticmethod
    def _distance_3d(a: VoxelCell, b: VoxelCell) -> float:
        """3D Euclidean distance."""
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)
    
    @staticmethod
    def _get_neighbors_3d(cell: VoxelCell) -> List[VoxelCell]:
        """Get 6-connected 3D neighbors."""
        x, y, z = cell
        neighbors = []
        for dx, dy, dz in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]:
            neighbors.append((x + dx, y + dy, z + dz))
        return neighbors
    
    def _world_to_voxel(
        self,
        pos: Tuple[float, float, float],
        origin_x: float,
        origin_y: float,
        origin_z: float,
    ) -> VoxelCell:
        """Convert world to voxel coordinates."""
        x = int((pos[0] - origin_x) / self.resolution)
        y = int((pos[1] - origin_y) / self.resolution)
        z = int((pos[2] - origin_z) / self.resolution)
        return (x, y, z)
    
    def _voxel_to_world(
        self,
        cell: VoxelCell,
        origin_x: float,
        origin_y: float,
        origin_z: float,
    ) -> Tuple[float, float, float]:
        """Convert voxel to world coordinates."""
        x = cell[0] * self.resolution + origin_x
        y = cell[1] * self.resolution + origin_y
        z = cell[2] * self.resolution + origin_z
        return (x, y, z)
