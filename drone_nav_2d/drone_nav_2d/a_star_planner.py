"""A* Path Planning Algorithm with Manhattan and Euclidean Heuristics.

Combines best of both heuristics:
- Manhattan: Better for grid-based navigation
- Euclidean: Better for continuous or complex environments

A* is fast for grid-based planning and provides optimal solutions
under admissible heuristics.
"""

import heapq
import math
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np


GridCell = Tuple[int, int]
VoxelCell = Tuple[int, int, int]


class AStarPlanner2D:
    """2D A* Path Planner with configurable heuristics."""

    def __init__(
        self,
        heuristic_weight_manhattan: float = 0.5,
        heuristic_weight_euclidean: float = 0.5,
        allow_diagonal: bool = True,
        collision_checker=None,
    ):
        """Initialize A* planner.
        
        Args:
            heuristic_weight_manhattan: Weight for Manhattan distance (0-1)
            heuristic_weight_euclidean: Weight for Euclidean distance (0-1)
            allow_diagonal: Allow diagonal movement (8-connected vs 4-connected)
            collision_checker: Function to check if node is free
        """
        self.heuristic_weight_manhattan = heuristic_weight_manhattan
        self.heuristic_weight_euclidean = heuristic_weight_euclidean
        self.allow_diagonal = allow_diagonal
        self.collision_checker = collision_checker
        
        # Normalize weights
        total_weight = heuristic_weight_manhattan + heuristic_weight_euclidean
        if total_weight > 0:
            self.heuristic_weight_manhattan /= total_weight
            self.heuristic_weight_euclidean /= total_weight
    
    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        get_neighbors: Callable[[GridCell], List[GridCell]],
        is_free: Callable[[GridCell], bool],
    ) -> List[Tuple[float, float]]:
        """Plan path using A* algorithm.
        
        Args:
            start: Start position (world coordinates)
            goal: Goal position (world coordinates)
            get_neighbors: Function to get valid neighbors of a cell
            is_free: Function to check if cell is collision-free
            
        Returns:
            Path from start to goal as list of world coordinates
        """
        self.collision_checker = is_free
        
        # Treat start and goal as grid cells for this implementation
        start_cell = start if isinstance(start, tuple) and isinstance(start[0], int) else (int(start[0]), int(start[1]))
        goal_cell = goal if isinstance(goal, tuple) and isinstance(goal[0], int) else (int(goal[0]), int(goal[1]))
        
        path_cells = self._a_star_search(start_cell, goal_cell, get_neighbors)
        
        return path_cells
    
    def _a_star_search(
        self,
        start: GridCell,
        goal: GridCell,
        get_neighbors: Callable[[GridCell], List[GridCell]],
    ) -> List[GridCell]:
        """Core A* search algorithm.
        
        Args:
            start: Start cell
            goal: Goal cell
            get_neighbors: Function to get neighbors
            
        Returns:
            Path as list of cells, or empty if no path found
        """
        # Priority queue: (f_score, counter, cell)
        counter = 0
        open_heap: List[Tuple[float, int, GridCell]] = []
        heapq.heappush(open_heap, (0.0, counter, start))
        counter += 1
        
        # Track visited cells
        came_from: Dict[GridCell, GridCell] = {}
        g_score: Dict[GridCell, float] = {start: 0.0}
        f_score: Dict[GridCell, float] = {start: self._heuristic(start, goal)}
        open_set = {start}
        
        max_expansions = 100000
        expansions = 0
        
        while open_heap and expansions < max_expansions:
            expansions += 1
            _, _, current = heapq.heappop(open_heap)
            
            if current not in open_set:
                continue
            
            open_set.remove(current)
            
            # Goal reached
            if current == goal:
                return self._reconstruct_path(came_from, current)
            
            # Explore neighbors
            for neighbor in get_neighbors(current):
                if not self.collision_checker(neighbor):
                    continue
                
                # Calculate step cost (diagonal moves cost more)
                step_cost = self._step_cost(current, neighbor)
                tentative_g = g_score[current] + step_cost
                
                # Found better path to neighbor
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    
                    if neighbor not in open_set:
                        open_set.add(neighbor)
                        heapq.heappush(
                            open_heap,
                            (f_score[neighbor], counter, neighbor)
                        )
                        counter += 1
        
        # No path found
        return []
    
    def _heuristic(self, a: GridCell, b: GridCell) -> float:
        """Hybrid heuristic combining Manhattan and Euclidean distances.
        
        - Manhattan: |x1-x2| + |y1-y2| (good for grid navigation)
        - Euclidean: sqrt((x1-x2)^2 + (y1-y2)^2) (good for continuous space)
        
        Combination provides better performance than either alone.
        """
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        
        manhattan = dx + dy
        euclidean = math.sqrt(dx * dx + dy * dy)
        
        return (
            self.heuristic_weight_manhattan * manhattan +
            self.heuristic_weight_euclidean * euclidean
        )
    
    def _step_cost(self, a: GridCell, b: GridCell) -> float:
        """Calculate movement cost between cells.
        
        - Cardinal (4-directional): cost = 1.0
        - Diagonal (8-directional): cost = sqrt(2) ≈ 1.414
        """
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        
        # Diagonal movement
        if dx > 0 and dy > 0:
            return math.sqrt(2)
        # Cardinal movement
        else:
            return 1.0
    
    @staticmethod
    def _reconstruct_path(
        came_from: Dict[GridCell, GridCell],
        current: GridCell,
    ) -> List[GridCell]:
        """Reconstruct path from start to goal."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


class AStarPlanner3D:
    """3D A* Path Planner for voxel-based navigation."""

    def __init__(
        self,
        heuristic_weight_manhattan: float = 0.5,
        heuristic_weight_euclidean: float = 0.5,
        collision_checker=None,
    ):
        """Initialize 3D A* planner.
        
        Args:
            heuristic_weight_manhattan: Weight for Manhattan distance
            heuristic_weight_euclidean: Weight for Euclidean distance
            collision_checker: Function to check if voxel is free
        """
        self.heuristic_weight_manhattan = heuristic_weight_manhattan
        self.heuristic_weight_euclidean = heuristic_weight_euclidean
        self.collision_checker = collision_checker
        
        # Normalize weights
        total_weight = heuristic_weight_manhattan + heuristic_weight_euclidean
        if total_weight > 0:
            self.heuristic_weight_manhattan /= total_weight
            self.heuristic_weight_euclidean /= total_weight
    
    def plan(
        self,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
        get_neighbors: Callable[[VoxelCell], List[VoxelCell]],
        is_free: Callable[[VoxelCell], bool],
    ) -> List[Tuple[float, float, float]]:
        """Plan 3D path using A*.
        
        Args:
            start: Start position (3D)
            goal: Goal position (3D)
            get_neighbors: Function to get valid neighbors
            is_free: Function to check collision
            
        Returns:
            Path as list of 3D positions
        """
        self.collision_checker = is_free
        
        start_cell = start if isinstance(start, tuple) and isinstance(start[0], int) else (int(start[0]), int(start[1]), int(start[2]))
        goal_cell = goal if isinstance(goal, tuple) and isinstance(goal[0], int) else (int(goal[0]), int(goal[1]), int(goal[2]))
        
        path_cells = self._a_star_search_3d(start_cell, goal_cell, get_neighbors)
        
        return path_cells
    
    def _a_star_search_3d(
        self,
        start: VoxelCell,
        goal: VoxelCell,
        get_neighbors: Callable[[VoxelCell], List[VoxelCell]],
    ) -> List[VoxelCell]:
        """Core 3D A* search.
        
        Args:
            start: Start voxel
            goal: Goal voxel
            get_neighbors: Function to get 26-connected neighbors
            
        Returns:
            Path as list of voxels
        """
        counter = 0
        open_heap: List[Tuple[float, int, VoxelCell]] = []
        heapq.heappush(open_heap, (0.0, counter, start))
        counter += 1
        
        came_from: Dict[VoxelCell, VoxelCell] = {}
        g_score: Dict[VoxelCell, float] = {start: 0.0}
        f_score: Dict[VoxelCell, float] = {start: self._heuristic_3d(start, goal)}
        open_set = {start}
        
        max_expansions = 500000
        expansions = 0
        
        while open_heap and expansions < max_expansions:
            expansions += 1
            _, _, current = heapq.heappop(open_heap)
            
            if current not in open_set:
                continue
            
            open_set.remove(current)
            
            if current == goal:
                return self._reconstruct_path_3d(came_from, current)
            
            for neighbor in get_neighbors(current):
                if not self.collision_checker(neighbor):
                    continue
                
                step_cost = self._step_cost_3d(current, neighbor)
                tentative_g = g_score[current] + step_cost
                
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic_3d(neighbor, goal)
                    
                    if neighbor not in open_set:
                        open_set.add(neighbor)
                        heapq.heappush(
                            open_heap,
                            (f_score[neighbor], counter, neighbor)
                        )
                        counter += 1
        
        return []
    
    def _heuristic_3d(self, a: VoxelCell, b: VoxelCell) -> float:
        """3D hybrid heuristic combining Manhattan and Euclidean."""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        dz = abs(a[2] - b[2])
        
        manhattan = dx + dy + dz
        euclidean = math.sqrt(dx * dx + dy * dy + dz * dz)
        
        return (
            self.heuristic_weight_manhattan * manhattan +
            self.heuristic_weight_euclidean * euclidean
        )
    
    def _step_cost_3d(self, a: VoxelCell, b: VoxelCell) -> float:
        """Calculate 3D movement cost (26-connectivity).
        
        - Cardinal (6 directions): 1.0
        - Face diagonal (12 directions): sqrt(2) ≈ 1.414
        - Space diagonal (8 directions): sqrt(3) ≈ 1.732
        """
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        dz = abs(a[2] - b[2])
        
        # Count non-zero differences
        num_directions = (dx > 0) + (dy > 0) + (dz > 0)
        
        if num_directions == 1:
            return 1.0  # Cardinal
        elif num_directions == 2:
            return math.sqrt(2)  # Face diagonal
        else:  # num_directions == 3
            return math.sqrt(3)  # Space diagonal
    
    @staticmethod
    def _reconstruct_path_3d(
        came_from: Dict[VoxelCell, VoxelCell],
        current: VoxelCell,
    ) -> List[VoxelCell]:
        """Reconstruct 3D path."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
