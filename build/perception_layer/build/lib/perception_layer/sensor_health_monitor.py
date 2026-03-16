#!/usr/bin/env python3
"""
Sensor Health Monitor Node
Monitors topic frequencies to determine if sensors are offline or failing.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu, LaserScan

class SensorHealthMonitor(Node):
    def __init__(self):
        super().__init__('sensor_health_monitor')
        self.get_logger().info("Sensor Health Monitor Started.")
        
        # Last received times
        self.last_imu_time = self.get_clock().now()
        self.last_scan_time = self.get_clock().now()

        # Subscriptions
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Health Publisher
        self.health_pub = self.create_publisher(String, '/sensor_status', 10)
        
        # Timer
        self.timer = self.create_timer(1.0, self.check_health)

    def imu_cb(self, msg):
        self.last_imu_time = self.get_clock().now()

    def scan_cb(self, msg):
        self.last_scan_time = self.get_clock().now()

    def check_health(self):
        now = self.get_clock().now()
        imu_diff = (now - self.last_imu_time).nanoseconds / 1e9
        scan_diff = (now - self.last_scan_time).nanoseconds / 1e9
        
        status = "OK"
        if imu_diff > 1.0:
            status = "IMU_FAULT"
        elif scan_diff > 2.0:
            status = "LIDAR_FAULT"
            
        msg = String()
        msg.data = status
        self.health_pub.publish(msg)
        if status != "OK":
            self.get_logger().warning(f"Sensor Issue Detected: {status}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
