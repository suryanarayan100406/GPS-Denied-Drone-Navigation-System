#!/usr/bin/env python3
"""
Noise Injection Node for Robustness Testing
Injects Gaussian noise into odometry to test EKF and Navigation resilience.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class NoiseInjectionNode(Node):
    def __init__(self):
        super().__init__('noise_injection_node')
        self.get_logger().info("Noise Injection Node Started.")
        
        # Subs and Pubs
        self.odom_sub = self.create_subscription(
            Odometry, '/lidar_odom_raw', self.odom_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/lidar_odom', 10)
        
        # Noise parameters
        self.noise_std_dev = 0.05  # 5cm standard deviation

    def odom_callback(self, msg: Odometry):
        noisy_msg = msg
        # Inject position noise
        noisy_msg.pose.pose.position.x += np.random.normal(0, self.noise_std_dev)
        noisy_msg.pose.pose.position.y += np.random.normal(0, self.noise_std_dev)
        # Publish noisy odom for EKF to filter out
        self.odom_pub.publish(noisy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NoiseInjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
