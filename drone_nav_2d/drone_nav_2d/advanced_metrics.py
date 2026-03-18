#!/usr/bin/env python3
"""
Advanced Metrics Evaluator - Comprehensive performance analysis
Features:
  - Real-time KPI tracking
  - Path optimality metrics
  - Energy efficiency analysis
  - Safety/collision metrics
  - Computational performance profiling
  - Automated evaluation reports
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool, String
import numpy as np
import json
import time
from pathlib import Path as PathlibPath
import psutil
import os


class AdvancedMetricsEvaluator(Node):
    """Tracks advanced metrics for performance evaluation."""
    
    def __init__(self):
        super().__init__('advanced_metrics_evaluator')
        
        # Parameters
        self.declare_parameter('output_dir', 'results')
        self.declare_parameter('evaluation_name', 'mission')
        
        self.output_dir = PathlibPath(self.get_parameter('output_dir').value)
        self.eval_name = self.get_parameter('evaluation_name').value
        self.output_dir.mkdir(exist_ok=True)
        
        # Subscriptions
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.create_subscription(PoseStamped, '/drone_pose', self.pose_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Float32, '/battery_level', self.battery_callback, 10)
        self.create_subscription(Bool, '/mission_complete', self.mission_complete_callback, 10)
        
        # Publication
        self.metrics_pub = self.create_publisher(String, '/evaluation_metrics', 10)
        
        # Metrics tracking
        self.metrics = {
            'mission_start_time': time.time(),
            'path_plans': [],
            'replans': 0,
            'trajectory_points': [],
            'min_obstacle_distance': float('inf'),
            'collision_count': 0,
            'commands_sent': 0,
            'battery_history': [],
            'energy_used': 0.0,
            'processing_times': [],
            'cpu_usage': [],
            'memory_usage': []
        }
        
        self.last_pose = None
        self.last_path = None
        self.mission_started = False
        self.mission_completed = False
        
        # Timer for periodic metric updates
        self.create_timer(1.0, self.update_performance_metrics)
        
        self.get_logger().info('Advanced Metrics Evaluator initialized')
    
    def path_callback(self, msg: Path):
        """Track planned paths."""
        if not self.mission_started:
            self.mission_started = True
        
        if self.last_path is not None:
            self.metrics['replans'] += 1
        
        path_data = {
            'timestamp': time.time(),
            'length': self._calculate_path_length(msg),
            'waypoint_count': len(msg.poses),
            'optimality_index': self._calculate_optimality(msg)
        }
        self.metrics['path_plans'].append(path_data)
        self.last_path = msg
    
    def pose_callback(self, msg: PoseStamped):
        """Track drone position."""
        pos = msg.pose.position
        self.metrics['trajectory_points'].append({
            'x': pos.x,
            'y': pos.y,
            'z': pos.z,
            't': time.time()
        })
        self.last_pose = msg
    
    def cmd_vel_callback(self, msg: Twist):
        """Track control commands."""
        self.metrics['commands_sent'] += 1
    
    def scan_callback(self, msg: LaserScan):
        """Track obstacle distances."""
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if not np.isnan(r) and not np.isinf(r)]
            if valid_ranges:
                min_dist = min(valid_ranges)
                self.metrics['min_obstacle_distance'] = min(
                    self.metrics['min_obstacle_distance'], min_dist
                )
                
                # Collision threshold (0.5m)
                if min_dist < 0.15:
                    self.metrics['collision_count'] += 1
    
    def battery_callback(self, msg: Float32):
        """Track battery level."""
        self.metrics['battery_history'].append({
            't': time.time(),
            'level': msg.data
        })
    
    def mission_complete_callback(self, msg: Bool):
        """Mark mission completion."""
        if msg.data:
            self.mission_completed = True
    
    def update_performance_metrics(self):
        """Update computational and performance metrics."""
        try:
            process = psutil.Process(os.getpid())
            self.metrics['cpu_usage'].append({
                't': time.time(),
                'percent': process.cpu_percent()
            })
            self.metrics['memory_usage'].append({
                't': time.time(),
                'mb': process.memory_info().rss / (1024 * 1024)
            })
        except:
            pass
    
    def _calculate_path_length(self, path: Path) -> float:
        """Calculate total path length."""
        if len(path.poses) < 2:
            return 0.0
        
        total = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i-1].pose.position
            p2 = path.poses[i].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            total += np.sqrt(dx*dx + dy*dy)
        return total
    
    def _calculate_optimality(self, path: Path) -> float:
        """Calculate path optimality (vs straight line)."""
        if len(path.poses) < 2:
            return 1.0
        
        start = path.poses[0].pose.position
        end = path.poses[-1].pose.position
        
        straight_dist = np.sqrt(
            (end.x - start.x)**2 + (end.y - start.y)**2
        )
        actual_dist = self._calculate_path_length(path)
        
        if straight_dist == 0:
            return 1.0
        return straight_dist / actual_dist if actual_dist > 0 else 0.0
    
    def generate_evaluation_report(self):
        """Generate comprehensive evaluation report."""
        if not self.metrics['trajectory_points']:
            self.get_logger().warn('No trajectory data collected')
            return
        
        mission_duration = time.time() - self.metrics['mission_start_time']
        
        report = {
            'evaluation_name': self.eval_name,
            'timestamp': time.time(),
            'mission_duration_seconds': mission_duration,
            'mission_completed': self.mission_completed,
            
            # Navigation metrics
            'navigation': {
                'total_replans': self.metrics['replans'],
                'path_plans_count': len(self.metrics['path_plans']),
                'initial_plan_length': self.metrics['path_plans'][0]['length'] if self.metrics['path_plans'] else 0.0,
                'avg_path_optimality': np.mean([p['optimality_index'] for p in self.metrics['path_plans']]) if self.metrics['path_plans'] else 0.0,
            },
            
            # Safety metrics
            'safety': {
                'min_obstacle_distance_m': float(self.metrics['min_obstacle_distance']),
                'collision_events': self.metrics['collision_count'],
                'safety_score': max(0, 100 - (self.metrics['collision_count'] * 10)),
            },
            
            # Energy metrics
            'energy': {
                'battery_readings': len(self.metrics['battery_history']),
                'initial_battery': self.metrics['battery_history'][0]['level'] if self.metrics['battery_history'] else 100.0,
                'final_battery': self.metrics['battery_history'][-1]['level'] if self.metrics['battery_history'] else 0.0,
                'energy_efficiency_score': self._calculate_energy_efficiency(),
            },
            
            # Performance metrics
            'performance': {
                'commands_sent': self.metrics['commands_sent'],
                'trajectory_points': len(self.metrics['trajectory_points']),
                'avg_cpu_usage_percent': np.mean([c['percent'] for c in self.metrics['cpu_usage']]) if self.metrics['cpu_usage'] else 0.0,
                'avg_memory_usage_mb': np.mean([m['mb'] for m in self.metrics['memory_usage']]) if self.metrics['memory_usage'] else 0.0,
                'peak_memory_mb': max([m['mb'] for m in self.metrics['memory_usage']]) if self.metrics['memory_usage'] else 0.0,
            },
            
            # Overall score
            'overall_score': self._calculate_overall_score(),
        }
        
        # Save to JSON
        report_path = self.output_dir / f'evaluation_report_{self.eval_name}.json'
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2)
        
        self.get_logger().info(f'Evaluation report saved to {report_path}')
        return report
    
    def _calculate_energy_efficiency(self) -> float:
        """Calculate energy efficiency score."""
        if not self.metrics['battery_history'] or not self.metrics['trajectory_points']:
            return 0.0
        
        distance = self._calculate_trajectory_distance()
        battery_used = self.metrics['battery_history'][0]['level'] - self.metrics['battery_history'][-1]['level']
        
        if battery_used <= 0 or distance == 0:
            return 0.0
        
        efficiency = distance / battery_used
        return min(100.0, efficiency * 10)
    
    def _calculate_trajectory_distance(self) -> float:
        """Calculate total trajectory distance."""
        if len(self.metrics['trajectory_points']) < 2:
            return 0.0
        
        total = 0.0
        for i in range(1, len(self.metrics['trajectory_points'])):
            p1 = self.metrics['trajectory_points'][i-1]
            p2 = self.metrics['trajectory_points'][i]
            dx = p2['x'] - p1['x']
            dy = p2['y'] - p1['y']
            total += np.sqrt(dx*dx + dy*dy)
        return total
    
    def _calculate_overall_score(self) -> float:
        """Calculate overall mission score (0-100)."""
        scores = []
        
        # Completion bonus
        if self.mission_completed:
            scores.append(50)
        
        # Safety score
        if self.metrics['collision_count'] == 0:
            scores.append(20)
        else:
            scores.append(max(0, 20 - (self.metrics['collision_count'] * 5)))
        
        # Energy efficiency
        energy_score = self._calculate_energy_efficiency()
        scores.append(min(20, energy_score / 5))
        
        # Path optimality
        if self.metrics['path_plans']:
            opt_avg = np.mean([p['optimality_index'] for p in self.metrics['path_plans']])
            scores.append(min(10, opt_avg * 10))
        
        return sum(scores)


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedMetricsEvaluator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Generating final evaluation report...')
        report = node.generate_evaluation_report()
        if report:
            node.get_logger().info(f'Overall Score: {report["overall_score"]:.1f}/100')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
