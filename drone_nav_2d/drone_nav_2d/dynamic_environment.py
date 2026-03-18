#!/usr/bin/env python3
"""
Dynamic Environment Manager - Adds runtime environmental challenges
Features:
  - Dynamic obstacle generation and movement
  - Terrain difficulty levels (easy/medium/hard)
  - Energy/battery modeling
  - Safety zones definition
  - Real-time constraint enforcement
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Pose2D
import numpy as np
import json
from datetime import datetime


class DynamicEnvironmentManager(Node):
    """Manages dynamic environmental challenges and constraints."""
    
    def __init__(self):
        super().__init__('dynamic_environment_manager')
        
        # Declare parameters
        self.declare_parameter('difficulty_level', 'easy')  # easy, medium, hard
        self.declare_parameter('enable_dynamic_obstacles', False)
        self.declare_parameter('battery_capacity', 3600.0)  # seconds at nominal power
        self.declare_parameter('power_consumption_rate', 0.5)  # J/s
        self.declare_parameter('terrain_roughness', 0.0)  # 0-1 scale
        
        # Get parameters
        self.difficulty = self.get_parameter('difficulty_level').value
        self.enable_dynamic = self.get_parameter('enable_dynamic_obstacles').value
        self.battery_capacity = self.get_parameter('battery_capacity').value
        self.power_rate = self.get_parameter('power_consumption_rate').value
        self.terrain_roughness = self.get_parameter('terrain_roughness').value
        
        # Publishers
        self.terrain_map_pub = self.create_publisher(OccupancyGrid, '/terrain_map', 10)
        self.battery_pub = self.create_publisher(Float32, '/battery_level', 10)
        self.energy_pub = self.create_publisher(Float32, '/energy_consumed', 10)
        self.constraints_pub = self.create_publisher(std_msgs.msg.String, '/environment_constraints', 10)
        
        # Environment state
        self.battery_level = 100.0
        self.energy_consumed = 0.0
        self.start_time = self.get_clock().now()
        self.dynamic_obstacles = []
        self.safety_zones = []
        self.terrain_difficulty_map = {}
        
        # Initialize environment based on difficulty
        self._initialize_environment()
        
        # Timer for environment updates
        self.timer = self.create_timer(0.5, self.update_environment)
        
        self.get_logger().info(f'Dynamic Environment Manager started - Difficulty: {self.difficulty}')
    
    def _initialize_environment(self):
        """Initialize environment based on difficulty level."""
        config = {
            'easy': {
                'obstacle_density': 0.15,
                'dynamic_obstacle_count': 0,
                'terrain_roughness': 0.0,
                'wind_speed': 0.0,
                'sensor_noise_std': 0.01,
                'processing_delay_ms': 10,
                'description': 'Simple open environment, minimal obstacles'
            },
            'medium': {
                'obstacle_density': 0.25,
                'dynamic_obstacle_count': 2,
                'terrain_roughness': 0.3,
                'wind_speed': 2.0,
                'sensor_noise_std': 0.05,
                'processing_delay_ms': 30,
                'description': 'Moderate complexity with moving obstacles'
            },
            'hard': {
                'obstacle_density': 0.40,
                'dynamic_obstacle_count': 5,
                'terrain_roughness': 0.7,
                'wind_speed': 5.0,
                'sensor_noise_std': 0.1,
                'processing_delay_ms': 50,
                'description': 'Dense obstacles, dynamic elements, harsh conditions'
            }
        }
        
        self.env_config = config.get(self.difficulty, config['easy'])
        
        # Generate dynamic obstacles if enabled
        if self.enable_dynamic:
            np.random.seed(42)
            for i in range(self.env_config['dynamic_obstacle_count']):
                obstacle = {
                    'id': i,
                    'x': np.random.uniform(-4, 4),
                    'y': np.random.uniform(-4, 4),
                    'vx': np.random.uniform(-0.5, 0.5),
                    'vy': np.random.uniform(-0.5, 0.5),
                    'radius': 0.3
                }
                self.dynamic_obstacles.append(obstacle)
        
        # Define safety zones (no-fly zones)
        self.safety_zones = [
            {'x': 0, 'y': -3.5, 'radius': 1.0, 'name': 'charging_station'},
            {'x': 4, 'y': 4, 'radius': 0.8, 'name': 'goal_zone'}
        ]
    
    def update_environment(self):
        """Update dynamic elements and publish status."""
        # Update battery level
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        energy_consumed = self.power_rate * elapsed
        self.battery_level = 100.0 - (energy_consumed / self.battery_capacity) * 100.0
        self.battery_level = max(0.0, self.battery_level)
        
        # Publish battery status
        battery_msg = Float32(data=self.battery_level)
        self.battery_pub.publish(battery_msg)
        
        energy_msg = Float32(data=energy_consumed)
        self.energy_pub.publish(energy_msg)
        
        # Update dynamic obstacles
        if self.dynamic_obstacles:
            for obstacle in self.dynamic_obstacles:
                obstacle['x'] += obstacle['vx'] * 0.5
                obstacle['y'] += obstacle['vy'] * 0.5
                
                # Bounce off boundaries
                if abs(obstacle['x']) > 4.5:
                    obstacle['vx'] *= -1
                if abs(obstacle['y']) > 4.5:
                    obstacle['vy'] *= -1
        
        # Generate constraint message
        constraints = {
            'timestamp': datetime.now().isoformat(),
            'battery_level_percent': float(self.battery_level),
            'energy_consumed_joules': float(energy_consumed),
            'difficulty_level': self.difficulty,
            'environment_config': self.env_config,
            'dynamic_obstacles': [
                {'id': o['id'], 'x': float(o['x']), 'y': float(o['y'])} 
                for o in self.dynamic_obstacles
            ],
            'safety_zones': self.safety_zones,
            'battery_warning': self.battery_level < 20.0,
            'battery_critical': self.battery_level < 5.0
        }
        
        # Check battery critical
        if self.battery_level < 5.0:
            self.get_logger().warn('CRITICAL: Battery below 5%!')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicEnvironmentManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
