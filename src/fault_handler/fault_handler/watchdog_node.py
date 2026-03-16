#!/usr/bin/env python3
"""
Watchdog Node (Fault Handler)
Monitors system health and enacts fallback behaviors (graceful degradation).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class WatchdogNode(Node):
    def __init__(self):
        super().__init__('watchdog_node')
        self.get_logger().info("Fault Handler Watchdog Node Started.")
        
        # Subs and Pubs
        self.health_sub = self.create_subscription(String, '/sensor_status', self.health_cb, 10)
        self.fault_pub = self.create_publisher(String, '/fault_log', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_override', 10) # override system

        self.current_fault = "OK"

    def health_cb(self, msg: String):
        if msg.data != self.current_fault:
            self.current_fault = msg.data
            fault_msg = String()
            
            if "LIDAR_FAULT" in self.current_fault:
                fault_msg.data = f"[{self.get_clock().now().to_msg().sec}] FAULT: LiDAR Dropped out. Switching to Visual Odometry (Simulated)."
                # Ideally, this would toggle EKF config via ROS2 param changes to trust Camera Odometry
                
            elif "IMU_FAULT" in self.current_fault:
                fault_msg.data = f"[{self.get_clock().now().to_msg().sec}] FAULT: IMU Dropped out. Emergency Hover."
                # Issue zero xy velocity, maintain current altitude
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                
            else:
                fault_msg.data = f"[{self.get_clock().now().to_msg().sec}] RECOVERY: Sensors nominal."

            self.fault_pub.publish(fault_msg)
            self.get_logger().error(fault_msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
