"""Probabilistic Roadmap (PRM) Path Planning Algorithm.

PRM builds a roadmap by sampling random configurations and connecting
nearby samples with straight-line segments. Provides fast planning after
roadmap construction.

Suitable for static environments and provides good solution quality.
"""

import math
import random
from typing import Dict, List, Optional, Set, Tuple

import numpy as np


class PRMPlanner2D:
    """2D Probabilistic Roadmap Planner.
    
    Builds a roadmap of randomly sampled nodes connected via local paths.
    """

    def __init__(
        self,
        num_samples: int = 500,
        connection_radius: float = 1.0,
        max_connections: int = 20,
        collision_checker=None,
    ):
        """Initialize PRM planner.
        
        Args:
            num_samples: Number of random samples to generate
            connection_radius: Radius within which to attempt connections
            max_connections: Maximum number of neighbors to connect
            collision_checker: Function to check collision (required)
        """
        self.num_samples = num_samples
        self.connection_radius = connection_radius
        self.max_connections = max_connections
        self.collision_checker = collision_checker
        
        # Roadmap storage
        self.nodes: List[Tuple[float, float]] = []
        self.edges: Dict[int, List[Tuple[int, float]]] = {}
        self.roadmap_built = False
        
    def build_roadmap(
        self,
        bounds: Tuple[float, float, float, float],
        blocked_regions: Optional[List] = None,
    ) -> bool:
        """Build PRM roadmap.
        
        Args:
            bounds: (xmin, xmax, ymin, ymax) of valid space
            blocked_regions: List of blocked regions to avoid
            
        Returns:
            True if roadmap was built successfully
        """
        self.nodes = []
        self.edges = {}
        
        x_min, x_max, y_min, y_max = bounds
        
        # Sample random configurations
        for _ in range(self.num_samples):
            config = (
                random.uniform(x_min, x_max),
                random.uniform(y_min, y_max),
            )
            
            # Check if configuration is valid (not in collision)
            if self.collision_checker(config):
                self.nodes.append(config)
        
        # Connect nearby nodes
        for i, node_i in enumerate(self.nodes):
            self.edges[i] = []
            
            # Find nearby nodes
            candidates = []
            for j, node_j in enumerate(self.nodes):
                if i >= j:
                    continue
                    
                dist = self._distance(node_i, node_j)
                if dist < self.connection_radius:
                    candidates.append((dist, j))
            
            # Sort by distance and connect to nearest neighbors
            candidates.sort()
            for _, j in candidates[:self.max_connections]:
                node_j = self.nodes[j]
                
                # Check if straight-line path is free
                if self._line_free(node_i, node_j):
                    dist = self._distance(node_i, node_j)
                    self.edges[i].append((j, dist))
                    
                    # Add reverse edge
                    if j not in self.edges:
                        self.edges[j] = []
                    self.edges[j].append((i, dist))
        
        self.roadmap_built = True
        return len(self.nodes) > 0
    
    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> List[Tuple[float, float]]:
        """Plan path using PRM roadmap.
        
        Args:
            start: Start position
            goal: Goal position
            
        Returns:
            Path from start to goal, or empty list if no path found
        """
        if not self.roadmap_built or not self.nodes:
            return []
        
        # Connect start to roadmap
        start_idx = len(self.nodes)
        start_neighbors = self._find_neighbors(start)
        
        if not start_neighbors:
            return []
        
        # Connect goal to roadmap
        goal_idx = len(self.nodes) + 1
        goal_neighbors = self._find_neighbors(goal)
        
        if not goal_neighbors:
            return []
        
        # Create temporary graph with start and goal
        all_nodes = self.nodes + [start, goal]
        temp_edges = {i: list(self.edges.get(i, [])) for i in range(len(self.nodes))}
        
        # Add start connections
        temp_edges[start_idx] = [(idx, self._distance(start, self.nodes[idx]))
                                  for idx in start_neighbors]
        
        # Add goal connections
        temp_edges[goal_idx] = [(idx, self._distance(goal, self.nodes[idx]))
                                 for idx in goal_neighbors]
        
        # Dijkstra search
        path_indices = self._dijkstra(temp_edges, start_idx, goal_idx, len(all_nodes))
        
        if not path_indices:
            return []
        
        return [all_nodes[i] for i in path_indices]
    
    def _find_neighbors(self, node: Tuple[float, float]) -> List[int]:
        """Find roadmap nodes within connection radius."""
        neighbors = []
        for i, pm_node in enumerate(self.nodes):
            dist = self._distance(node, pm_node)
            if dist < self.connection_radius and self._line_free(node, pm_node):
                neighbors.append(i)
        return neighbors
    
    def _line_free(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        samples: int = 10,
    ) -> bool:
        """Check if straight line between points is collision-free."""
        for i in range(samples + 1):
            t = i / max(1, samples)
            point = (
                p1[0] + t * (p2[0] - p1[0]),
                p1[1] + t * (p2[1] - p1[1]),
            )
            if not self.collision_checker(point):
                return False
        return True
    
    def _dijkstra(
        self,
        graph: Dict[int, List[Tuple[int, float]]],
        start: int,
        goal: int,
        num_nodes: int,
    ) -> List[int]:
        """Dijkstra shortest path algorithm."""
        import heapq
        
        dist = {i: float('inf') for i in range(num_nodes)}
        dist[start] = 0
        came_from = {}
        
        heap = [(0, start)]
        visited = set()
        
        while heap:
            current_dist, current = heapq.heappop(heap)
            
            if current in visited:
                continue
            visited.add(current)
            
            if current == goal:
                # Reconstruct path
                path = [goal]
                while goal in came_from:
                    goal = came_from[goal]
                    path.append(goal)
                path.reverse()
                return path
            
            for neighbor, edge_dist in graph.get(current, []):
                if neighbor in visited:
                    continue
                
                new_dist = current_dist + edge_dist
                if new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    came_from[neighbor] = current
                    heapq.heappush(heap, (new_dist, neighbor))
        
        return []
    
    @staticmethod
    def _distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Euclidean distance between points."""
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx * dx + dy * dy)


class PRMPlanner3D:
    """3D Probabilistic Roadmap Planner.
    
    Extends 2D PRM to 3D configuration space.
    """

    def __init__(
        self,
        num_samples: int = 800,
        connection_radius: float = 1.5,
        max_connections: int = 25,
        collision_checker=None,
    ):
        """Initialize 3D PRM planner.
        
        Args:
            num_samples: Number of random samples to generate
            connection_radius: Radius within which to attempt connections
            max_connections: Maximum number of neighbors to connect
            collision_checker: Function to check collision (required)
        """
        self.num_samples = num_samples
        self.connection_radius = connection_radius
        self.max_connections = max_connections
        self.collision_checker = collision_checker
        
        # Roadmap storage
        self.nodes: List[Tuple[float, float, float]] = []
        self.edges: Dict[int, List[Tuple[int, float]]] = {}
        self.roadmap_built = False
    
    def build_roadmap(
        self,
        bounds: Tuple[float, float, float, float, float, float],
        blocked_regions: Optional[List] = None,
    ) -> bool:
        """Build 3D PRM roadmap.
        
        Args:
            bounds: (xmin, xmax, ymin, ymax, zmin, zmax)
            blocked_regions: List of blocked regions to avoid
            
        Returns:
            True if roadmap built successfully
        """
        self.nodes = []
        self.edges = {}
        
        x_min, x_max, y_min, y_max, z_min, z_max = bounds
        
        # Sample configurations
        for _ in range(self.num_samples):
            config = (
                random.uniform(x_min, x_max),
                random.uniform(y_min, y_max),
                random.uniform(z_min, z_max),
            )
            
            if self.collision_checker(config):
                self.nodes.append(config)
        
        # Connect nearby nodes
        for i, node_i in enumerate(self.nodes):
            self.edges[i] = []
            
            candidates = []
            for j, node_j in enumerate(self.nodes):
                if i >= j:
                    continue
                dist = self._distance(node_i, node_j)
                if dist < self.connection_radius:
                    candidates.append((dist, j))
            
            candidates.sort()
            for _, j in candidates[:self.max_connections]:
                node_j = self.nodes[j]
                if self._line_free(node_i, node_j):
                    dist = self._distance(node_i, node_j)
                    self.edges[i].append((j, dist))
                    if j not in self.edges:
                        self.edges[j] = []
                    self.edges[j].append((i, dist))
        
        self.roadmap_built = True
        return len(self.nodes) > 0
    
    def plan(
        self,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
    ) -> List[Tuple[float, float, float]]:
        """Plan 3D path using PRM."""
        if not self.roadmap_built or not self.nodes:
            return []
        
        start_neighbors = self._find_neighbors(start)
        if not start_neighbors:
            return []
        
        goal_neighbors = self._find_neighbors(goal)
        if not goal_neighbors:
            return []
        
        all_nodes = self.nodes + [start, goal]
        temp_edges = {i: list(self.edges.get(i, [])) for i in range(len(self.nodes))}
        
        start_idx = len(self.nodes)
        goal_idx = len(self.nodes) + 1
        
        temp_edges[start_idx] = [(idx, self._distance(start, self.nodes[idx]))
                                  for idx in start_neighbors]
        temp_edges[goal_idx] = [(idx, self._distance(goal, self.nodes[idx]))
                                 for idx in goal_neighbors]
        
        path_indices = self._dijkstra(temp_edges, start_idx, goal_idx, len(all_nodes))
        
        if not path_indices:
            return []
        
        return [all_nodes[i] for i in path_indices]
    
    def _find_neighbors(self, node: Tuple[float, float, float]) -> List[int]:
        """Find nearby roadmap nodes."""
        neighbors = []
        for i, pm_node in enumerate(self.nodes):
            dist = self._distance(node, pm_node)
            if dist < self.connection_radius and self._line_free(node, pm_node):
                neighbors.append(i)
        return neighbors
    
    def _line_free(
        self,
        p1: Tuple[float, float, float],
        p2: Tuple[float, float, float],
        samples: int = 15,
    ) -> bool:
        """Check collision-free path in 3D."""
        for i in range(samples + 1):
            t = i / max(1, samples)
            point = (
                p1[0] + t * (p2[0] - p1[0]),
                p1[1] + t * (p2[1] - p1[1]),
                p1[2] + t * (p2[2] - p1[2]),
            )
            if not self.collision_checker(point):
                return False
        return True
    
    def _dijkstra(
        self,
        graph: Dict[int, List[Tuple[int, float]]],
        start: int,
        goal: int,
        num_nodes: int,
    ) -> List[int]:
        """Dijkstra shortest path in 3D."""
        import heapq
        
        dist = {i: float('inf') for i in range(num_nodes)}
        dist[start] = 0
        came_from = {}
        
        heap = [(0, start)]
        visited = set()
        
        while heap:
            current_dist, current = heapq.heappop(heap)
            
            if current in visited:
                continue
            visited.add(current)
            
            if current == goal:
                path = [goal]
                while goal in came_from:
                    goal = came_from[goal]
                    path.append(goal)
                path.reverse()
                return path
            
            for neighbor, edge_dist in graph.get(current, []):
                if neighbor in visited:
                    continue
                new_dist = current_dist + edge_dist
                if new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    came_from[neighbor] = current
                    heapq.heappush(heap, (new_dist, neighbor))
        
        return []
    
    @staticmethod
    def _distance(
        p1: Tuple[float, float, float],
        p2: Tuple[float, float, float],
    ) -> float:
        """3D Euclidean distance."""
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        dz = p1[2] - p2[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)
