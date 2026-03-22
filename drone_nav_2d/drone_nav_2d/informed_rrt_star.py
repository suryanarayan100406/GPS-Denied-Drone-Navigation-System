"""Informed RRT* Path Planning Algorithm.

Informed RRT* improves upon RRT* by using informed sampling informed by
previous solutions. It provides high-quality paths more quickly than standard RRT*.

Key improvements:
- Ellipse-based informed sampling
- Asymptotically optimal paths
- Better solution quality than RRT
"""

import math
import random
from typing import Dict, List, Optional, Tuple

import numpy as np


class InformedRRTStar2D:
    """2D Informed RRT* Planner for asymptotically optimal path planning."""

    def __init__(
        self,
        max_iterations: int = 3000,
        step_size: float = 0.5,
        goal_sample_rate: float = 0.15,
        rewire_radius_factor: float = 30.0,
        collision_checker=None,
    ):
        """Initialize Informed RRT* planner.
        
        Args:
            max_iterations: Maximum planning iterations
            step_size: Maximum extension distance
            goal_sample_rate: Probability of sampling goal
            rewire_radius_factor: Factor for rewiring radius
            collision_checker: Function to check collision
        """
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.rewire_radius_factor = rewire_radius_factor
        self.collision_checker = collision_checker
        
        self.best_cost = float('inf')
        self.best_path: List[Tuple[float, float]] = []
    
    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        bounds: Tuple[float, float, float, float],
    ) -> List[Tuple[float, float]]:
        """Plan path using Informed RRT*.
        
        Args:
            start: Start configuration
            goal: Goal configuration
            bounds: (xmin, xmax, ymin, ymax)
            
        Returns:
            Path from start to goal
        """
        if not self.collision_checker(start) or not self.collision_checker(goal):
            return []
        
        x_min, x_max, y_min, y_max = bounds
        
        # Tree representation
        nodes = {0: start}
        edges: Dict[int, List[int]] = {0: []}
        parents: Dict[int, int] = {}
        costs: Dict[int, float] = {0: 0.0}
        
        best_node = None
        self.best_cost = float('inf')
        self.best_path = []
        
        for iteration in range(self.max_iterations):
            # Sample random point or use informed sampling
            if random.random() < self.goal_sample_rate:
                sample = goal
            else:
                sample = self._informed_sample(
                    start, goal, self.best_cost, (x_min, x_max, y_min, y_max)
                )
            
            # Find nearest node in tree
            nearest_idx = min(
                nodes.keys(),
                key=lambda n: self._distance(nodes[n], sample),
            )
            nearest_node = nodes[nearest_idx]
            
            # Steer toward sample
            new_node = self._steer(nearest_node, sample)
            
            # Check collision
            if not self._path_free(nearest_node, new_node):
                continue
            
            # Add node to tree
            new_idx = len(nodes)
            nodes[new_idx] = new_node
            edges[new_idx] = []
            costs[new_idx] = costs[nearest_idx] + self._distance(nearest_node, new_node)
            parents[new_idx] = nearest_idx
            
            # Rewire: find nodes within rewiring radius
            rewire_radius = self._calculate_rewire_radius(len(nodes))
            
            # Try to rewire existing nodes
            candidates = self._find_nearby_nodes(new_node, rewire_radius, nodes)
            
            for idx in candidates:
                if idx == new_idx or idx == nearest_idx:
                    continue
                
                candidate_node = nodes[idx]
                dist = self._distance(new_node, candidate_node)
                new_cost = costs[new_idx] + dist
                
                # Check if this is a better path to candidate
                if new_cost < costs[idx] and self._path_free(new_node, candidate_node):
                    old_parent = parents[idx]
                    parents[idx] = new_idx
                    costs[idx] = new_cost
                    edges[new_idx].append(idx)
                    
                    if old_parent in edges[old_parent]:
                        edges[old_parent].remove(idx)
                
                # Check if candidate provides better path to new_node
                candidate_cost = costs[idx] + dist
                if candidate_cost < costs[new_idx] and self._path_free(candidate_node, new_node):
                    parents[new_idx] = idx
                    costs[new_idx] = candidate_cost
                    edges[idx].append(new_idx)
                    
                    if nearest_idx in edges.get(nearest_idx, []):
                        edges[nearest_idx].remove(new_idx)
            
            # Check if goal is reached
            goal_dist = self._distance(new_node, goal)
            if goal_dist < self.step_size and self._path_free(new_node, goal):
                goal_idx = len(nodes)
                nodes[goal_idx] = goal
                goal_cost = costs[new_idx] + goal_dist
                
                if goal_cost < self.best_cost:
                    self.best_cost = goal_cost
                    parents[goal_idx] = new_idx
                    best_node = goal_idx
                    self.best_path = self._reconstruct_path(nodes, parents, goal_idx)
        
        return self.best_path
    
    def _informed_sample(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        best_cost: float,
        bounds: Tuple[float, float, float, float],
    ) -> Tuple[float, float]:
        """Sample from informed set using ellipse."""
        if best_cost == float('inf'):
            # Use uniform sampling in bounds
            x_min, x_max, y_min, y_max = bounds
            return (
                random.uniform(x_min, x_max),
                random.uniform(y_min, y_max),
            )
        
        # Create ellipse with start/goal as foci
        c_min = self._distance(start, goal)
        if c_min == 0 or best_cost < c_min:
            return goal
        
        # Ellipse semi-major and semi-minor axes
        a = best_cost / 2.0
        b = math.sqrt(a * a - (c_min / 2.0) ** 2)
        
        # Sample from ellipse
        r1 = random.random()
        r2 = random.random()
        x = r1 * 2 * a - a
        y = (math.sqrt(1 - (x / a) ** 2) if abs(x) < a else 0) * b * (1 if r2 < 0.5 else -1)
        
        # Transform to world coordinates
        center = ((start[0] + goal[0]) / 2, (start[1] + goal[1]) / 2)
        angle = math.atan2(goal[1] - start[1], goal[0] - start[0])
        
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        world_x = center[0] + x * cos_a - y * sin_a
        world_y = center[1] + x * sin_a + y * cos_a
        
        return (world_x, world_y)
    
    def _find_nearby_nodes(
        self,
        point: Tuple[float, float],
        radius: float,
        nodes: Dict[int, Tuple[float, float]],
    ) -> List[int]:
        """Find nodes within radius."""
        nearby = []
        for idx, node in nodes.items():
            if self._distance(point, node) < radius:
                nearby.append(idx)
        return nearby
    
    def _steer(
        self,
        from_node: Tuple[float, float],
        to_node: Tuple[float, float],
    ) -> Tuple[float, float]:
        """Steer from from_node toward to_node."""
        dist = self._distance(from_node, to_node)
        if dist < self.step_size:
            return to_node
        
        ratio = self.step_size / dist
        return (
            from_node[0] + ratio * (to_node[0] - from_node[0]),
            from_node[1] + ratio * (to_node[1] - from_node[1]),
        )
    
    def _path_free(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        samples: int = 10,
    ) -> bool:
        """Check if path is collision-free."""
        for i in range(samples + 1):
            t = i / max(1, samples)
            point = (
                p1[0] + t * (p2[0] - p1[0]),
                p1[1] + t * (p2[1] - p1[1]),
            )
            if not self.collision_checker(point):
                return False
        return True
    
    def _calculate_rewire_radius(self, tree_size: int) -> float:
        """Calculate dynamic rewiring radius."""
        return self.rewire_radius_factor * math.sqrt(math.log(tree_size) / tree_size)
    
    def _reconstruct_path(
        self,
        nodes: Dict[int, Tuple[float, float]],
        parents: Dict[int, int],
        goal_idx: int,
    ) -> List[Tuple[float, float]]:
        """Reconstruct path from tree."""
        path = []
        current = goal_idx
        while current in nodes:
            path.append(nodes[current])
            if current not in parents:
                break
            current = parents[current]
        path.reverse()
        return path
    
    @staticmethod
    def _distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Euclidean distance."""
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx * dx + dy * dy)


class InformedRRTStar3D:
    """3D Informed RRT* Planner."""

    def __init__(
        self,
        max_iterations: int = 4000,
        step_size: float = 0.6,
        goal_sample_rate: float = 0.12,
        rewire_radius_factor: float = 35.0,
        collision_checker=None,
    ):
        """Initialize 3D Informed RRT*."""
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.rewire_radius_factor = rewire_radius_factor
        self.collision_checker = collision_checker
        
        self.best_cost = float('inf')
        self.best_path: List[Tuple[float, float, float]] = []
    
    def plan(
        self,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
        bounds: Tuple[float, float, float, float, float, float],
    ) -> List[Tuple[float, float, float]]:
        """Plan 3D path using Informed RRT*."""
        if not self.collision_checker(start) or not self.collision_checker(goal):
            return []
        
        x_min, x_max, y_min, y_max, z_min, z_max = bounds
        
        nodes = {0: start}
        edges: Dict[int, List[int]] = {0: []}
        parents: Dict[int, int] = {}
        costs: Dict[int, float] = {0: 0.0}
        
        self.best_cost = float('inf')
        self.best_path = []
        
        for iteration in range(self.max_iterations):
            if random.random() < self.goal_sample_rate:
                sample = goal
            else:
                sample = (
                    random.uniform(x_min, x_max),
                    random.uniform(y_min, y_max),
                    random.uniform(z_min, z_max),
                )
            
            nearest_idx = min(
                nodes.keys(),
                key=lambda n: self._distance(nodes[n], sample),
            )
            nearest_node = nodes[nearest_idx]
            
            new_node = self._steer(nearest_node, sample)
            
            if not self._path_free(nearest_node, new_node):
                continue
            
            new_idx = len(nodes)
            nodes[new_idx] = new_node
            edges[new_idx] = []
            costs[new_idx] = costs[nearest_idx] + self._distance(nearest_node, new_node)
            parents[new_idx] = nearest_idx
            
            # Rewiring
            rewire_radius = self._calculate_rewire_radius(len(nodes))
            candidates = self._find_nearby_nodes(new_node, rewire_radius, nodes)
            
            for idx in candidates:
                if idx == new_idx or idx == nearest_idx:
                    continue
                
                candidate_node = nodes[idx]
                dist = self._distance(new_node, candidate_node)
                new_cost = costs[new_idx] + dist
                
                if new_cost < costs[idx] and self._path_free(new_node, candidate_node):
                    old_parent = parents[idx]
                    parents[idx] = new_idx
                    costs[idx] = new_cost
                    edges[new_idx].append(idx)
                    if old_parent in edges.get(old_parent, []):
                        edges[old_parent].remove(idx)
                
                candidate_cost = costs[idx] + dist
                if candidate_cost < costs[new_idx] and self._path_free(candidate_node, new_node):
                    parents[new_idx] = idx
                    costs[new_idx] = candidate_cost
                    edges[idx].append(new_idx)
                    if nearest_idx in edges.get(nearest_idx, []):
                        edges[nearest_idx].remove(new_idx)
            
            # Check goal
            goal_dist = self._distance(new_node, goal)
            if goal_dist < self.step_size and self._path_free(new_node, goal):
                goal_idx = len(nodes)
                nodes[goal_idx] = goal
                goal_cost = costs[new_idx] + goal_dist
                
                if goal_cost < self.best_cost:
                    self.best_cost = goal_cost
                    parents[goal_idx] = new_idx
                    self.best_path = self._reconstruct_path(nodes, parents, goal_idx)
        
        return self.best_path
    
    def _find_nearby_nodes(
        self,
        point: Tuple[float, float, float],
        radius: float,
        nodes: Dict[int, Tuple[float, float, float]],
    ) -> List[int]:
        """Find nearby 3D nodes."""
        nearby = []
        for idx, node in nodes.items():
            if self._distance(point, node) < radius:
                nearby.append(idx)
        return nearby
    
    def _steer(
        self,
        from_node: Tuple[float, float, float],
        to_node: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        """Steer in 3D."""
        dist = self._distance(from_node, to_node)
        if dist < self.step_size:
            return to_node
        
        ratio = self.step_size / dist
        return (
            from_node[0] + ratio * (to_node[0] - from_node[0]),
            from_node[1] + ratio * (to_node[1] - from_node[1]),
            from_node[2] + ratio * (to_node[2] - from_node[2]),
        )
    
    def _path_free(
        self,
        p1: Tuple[float, float, float],
        p2: Tuple[float, float, float],
        samples: int = 15,
    ) -> bool:
        """Check 3D collision-free path."""
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
    
    def _calculate_rewire_radius(self, tree_size: int) -> float:
        """Calculate rewiring radius."""
        return self.rewire_radius_factor * math.sqrt(math.log(tree_size) / tree_size)
    
    def _reconstruct_path(
        self,
        nodes: Dict[int, Tuple[float, float, float]],
        parents: Dict[int, int],
        goal_idx: int,
    ) -> List[Tuple[float, float, float]]:
        """Reconstruct 3D path."""
        path = []
        current = goal_idx
        while current in nodes:
            path.append(nodes[current])
            if current not in parents:
                break
            current = parents[current]
        path.reverse()
        return path
    
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
