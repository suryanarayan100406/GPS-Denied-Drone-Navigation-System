import math
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


@dataclass
class PIDState:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    integral: float = 0.0
    prev_error: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return self.kp * error
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class DroneController(Node):
    def __init__(self) -> None:
        super().__init__('drone_controller')

        self.declare_parameter('pose_topic', '/webots/drone/pose')
        self.declare_parameter('use_ground_truth_fallback', True)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('max_velocity_xy', 1.2)
        self.declare_parameter('max_velocity_z', 0.6)
        self.declare_parameter('fixed_altitude', 0.5)

        self.declare_parameter('kp_x', 1.2)
        self.declare_parameter('ki_x', 0.05)
        self.declare_parameter('kd_x', 0.2)
        self.declare_parameter('kp_y', 1.2)
        self.declare_parameter('ki_y', 0.05)
        self.declare_parameter('kd_y', 0.2)
        self.declare_parameter('kp_z', 1.5)
        self.declare_parameter('ki_z', 0.05)
        self.declare_parameter('kd_z', 0.25)
        self.declare_parameter('integral_limit_xy', 1.0)
        self.declare_parameter('integral_limit_z', 0.8)

        pose_topic = self.get_parameter('pose_topic').value
        self.use_ground_truth_fallback = bool(self.get_parameter('use_ground_truth_fallback').value)
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.max_velocity_xy = float(self.get_parameter('max_velocity_xy').value)
        self.max_velocity_z = float(self.get_parameter('max_velocity_z').value)
        self.fixed_altitude = float(self.get_parameter('fixed_altitude').value)

        self.pid_x = PIDState(
            kp=float(self.get_parameter('kp_x').value),
            ki=float(self.get_parameter('ki_x').value),
            kd=float(self.get_parameter('kd_x').value),
            integral_limit=float(self.get_parameter('integral_limit_xy').value),
        )
        self.pid_y = PIDState(
            kp=float(self.get_parameter('kp_y').value),
            ki=float(self.get_parameter('ki_y').value),
            kd=float(self.get_parameter('kd_y').value),
            integral_limit=float(self.get_parameter('integral_limit_xy').value),
        )
        self.pid_z = PIDState(
            kp=float(self.get_parameter('kp_z').value),
            ki=float(self.get_parameter('ki_z').value),
            kd=float(self.get_parameter('kd_z').value),
            integral_limit=float(self.get_parameter('integral_limit_z').value),
        )

        self.current_pose: Optional[PoseStamped] = None
        self.path_points: List[PoseStamped] = []
        self.current_index = 0
        self.goal_reached = False

        self.obstacle_override = False
        self.avoidance_cmd = Twist()
        self.last_avoidance_time = self.get_clock().now()

        self.sim_pose_x = -4.0
        self.sim_pose_y = 0.0
        self.sim_pose_z = self.fixed_altitude

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone_pose', 10)
        self.traj_pub = self.create_publisher(Path, '/drone_trajectory', 10)
        self.mission_complete_pub = self.create_publisher(Bool, '/mission_complete', 10)

        self.create_subscription(Path, '/planned_path', self._on_path, 10)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)
        self.create_subscription(Twist, '/avoidance_cmd_vel', self._on_avoidance_cmd, 10)
        self.create_subscription(Bool, '/obstacle_detected', self._on_obstacle, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.traj_msg = Path()
        self.traj_msg.header.frame_id = 'map'

        self.last_control_time = self.get_clock().now()
        self.timer = self.create_timer(max(0.02, 1.0 / control_rate_hz), self._control_loop)

        self.get_logger().info('Drone controller initialized.')

    def _on_path(self, msg: Path) -> None:
        self.path_points = msg.poses
        self.current_index = 0
        self.goal_reached = False
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.get_logger().info(f'Received path with {len(self.path_points)} waypoints.')

    def _on_pose(self, msg: PoseStamped) -> None:
        self.current_pose = msg

    def _on_avoidance_cmd(self, msg: Twist) -> None:
        self.avoidance_cmd = msg
        self.last_avoidance_time = self.get_clock().now()

    def _on_obstacle(self, msg: Bool) -> None:
        self.obstacle_override = msg.data

    def _active_pose(self) -> PoseStamped:
        if self.current_pose is not None:
            return self.current_pose

        fallback = PoseStamped()
        fallback.header.stamp = self.get_clock().now().to_msg()
        fallback.header.frame_id = 'map'
        fallback.pose.position.x = self.sim_pose_x
        fallback.pose.position.y = self.sim_pose_y
        fallback.pose.position.z = self.sim_pose_z
        fallback.pose.orientation.w = 1.0
        return fallback

    def _control_loop(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now

        pose_msg = self._active_pose()
        if self.current_pose is None and not self.use_ground_truth_fallback:
            return

        self.pose_pub.publish(pose_msg)
        self._publish_tf(pose_msg)
        self._update_trajectory(pose_msg)

        if self.goal_reached or not self.path_points:
            self._publish_stop()
            return

        if self.current_index >= len(self.path_points):
            self._mark_goal_reached()
            self._publish_stop()
            return

        target = self.path_points[self.current_index].pose.position
        current = pose_msg.pose.position

        err_x = target.x - current.x
        err_y = target.y - current.y
        err_z = self.fixed_altitude - current.z

        dist_xy = math.hypot(err_x, err_y)
        if dist_xy <= self.waypoint_tolerance:
            self.current_index += 1
            if self.current_index >= len(self.path_points):
                self._mark_goal_reached()
            return

        cmd = Twist()
        cmd.linear.x = self.pid_x.update(err_x, dt)
        cmd.linear.y = self.pid_y.update(err_y, dt)
        cmd.linear.z = self.pid_z.update(err_z, dt)

        cmd.linear.x = max(-self.max_velocity_xy, min(self.max_velocity_xy, cmd.linear.x))
        cmd.linear.y = max(-self.max_velocity_xy, min(self.max_velocity_xy, cmd.linear.y))
        cmd.linear.z = max(-self.max_velocity_z, min(self.max_velocity_z, cmd.linear.z))

        if self.obstacle_override:
            age = (now - self.last_avoidance_time).nanoseconds / 1e9
            if age < 0.4:
                cmd.linear.x = self.avoidance_cmd.linear.x
                cmd.linear.y = self.avoidance_cmd.linear.y

        self.cmd_pub.publish(cmd)
        self._integrate_fallback_pose(cmd, dt)

    def _integrate_fallback_pose(self, cmd: Twist, dt: float) -> None:
        if self.current_pose is not None:
            return
        if dt <= 0.0:
            return
        self.sim_pose_x += cmd.linear.x * dt
        self.sim_pose_y += cmd.linear.y * dt
        self.sim_pose_z += cmd.linear.z * dt

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _mark_goal_reached(self) -> None:
        if self.goal_reached:
            return
        self.goal_reached = True
        msg = Bool()
        msg.data = True
        self.mission_complete_pub.publish(msg)
        self.get_logger().info('Mission complete: goal reached.')

    def _update_trajectory(self, pose_msg: PoseStamped) -> None:
        pose_copy = PoseStamped()
        pose_copy.header = pose_msg.header
        pose_copy.pose = pose_msg.pose
        self.traj_msg.header.stamp = pose_msg.header.stamp
        self.traj_msg.poses.append(pose_copy)
        self.traj_pub.publish(self.traj_msg)

    def _publish_tf(self, pose_msg: PoseStamped) -> None:
        t = TransformStamped()
        t.header.stamp = pose_msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'drone_base'
        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z
        t.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
