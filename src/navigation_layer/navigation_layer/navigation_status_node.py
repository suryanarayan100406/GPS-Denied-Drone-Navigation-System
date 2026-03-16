#!/usr/bin/env python3
"""
Navigation Status Node
Publishes waypoint status and navigation metrics.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NavigationStatusNode(Node):
    def __init__(self):
        super().__init__('navigation_status_node')
        self.get_logger().info("Navigation Status Node Started.")
        self.pub = self.create_publisher(String, '/navigation_status', 10)
        
        # publish simple alive pulse for now
        self.timer = self.create_timer(2.0, self.timer_cb)

    def timer_cb(self):
        msg = String()
        msg.data = "NAV_IDLE: Awaiting goals."
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
