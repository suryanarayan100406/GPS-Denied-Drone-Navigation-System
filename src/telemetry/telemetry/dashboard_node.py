#!/usr/bin/env python3
"""
Telemetry Dashboard Node
Aggregates metrics and serves a local interface for judges via rosbridge.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import csv
import os

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        self.get_logger().info("Telemetry Dashboard Node Started. Providing data to Web UI.")
        
        # Subscribe to metrics
        self.pid_sub = self.create_subscription(Float32MultiArray, '/pid_telemetry', self.pid_cb, 10)
        self.fault_sub = self.create_subscription(String, '/fault_log', self.fault_cb, 10)
        
        # CSV Logging
        self.csv_path = os.path.expanduser('~/drone_ws/docs/performance_metrics.csv')
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Timestamp', 'Z_Error', 'Z_Integral', 'Z_Derivative', 'Fault_Events'])

        self.last_fault = ""

    def fault_cb(self, msg):
        self.last_fault = msg.data

    def pid_cb(self, msg: Float32MultiArray):
        # We assume data is [z_error, z_integral, z_derivative]
        if len(msg.data) >= 3:
            try:
                with open(self.csv_path, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([self.get_clock().now().to_msg().sec, msg.data[0], msg.data[1], msg.data[2], self.last_fault])
            except Exception as e:
                self.get_logger().error(f"CSV Logging Failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
