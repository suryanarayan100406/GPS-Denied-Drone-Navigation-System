"""
Headless launch without Webots simulator - for testing navigation on WSL2 where GUI is limited.
This launches all navigation nodes with RViz for visualization and rosbag for recording.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('drone_nav_2d')
    params_file = os.path.join(pkg_share, 'config', 'nav_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'drone_nav.rviz')
    urdf_path = os.path.join(pkg_share, 'urdf', 'drone.urdf')

    bag_output = LaunchConfiguration('bag_output')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    # Map publisher node
    map_publisher = Node(
        package='drone_nav_2d',
        executable='map_publisher',
        name='map_publisher',
        output='screen',
        parameters=[params_file],
    )

    # Path planner node
    planner = Node(
        package='drone_nav_2d',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[params_file],
    )

    # Create synthetic drone pose publisher for testing (since no Webots)
    synthetic_pose = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '-1',
            '/drone_pose',
            'geometry_msgs/PoseStamped',
            '{header: {frame_id: "map"}, pose: {position: {x: -4.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}'
        ],
        output='screen',
    )

    # Drone controller node
    controller = Node(
        package='drone_nav_2d',
        executable='drone_controller',
        name='drone_controller',
        output='screen',
        parameters=[params_file],
    )

    # Obstacle avoidance node
    avoidance = Node(
        package='drone_nav_2d',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[params_file],
    )

    # Metrics logger node
    metrics = Node(
        package='drone_nav_2d',
        executable='metrics_logger',
        name='metrics_logger',
        output='screen',
        parameters=[params_file],
    )

    # RViz2 for visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # rosbag2 recording for video/data capture
    rosbag = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'record',
            '-o',
            bag_output,
            '/map',
            '/planned_path',
            '/drone_pose',
            '/drone_trajectory',
            '/cmd_vel',
            '/obstacle_detected',
            '/replan_event',
            '/min_obstacle_distance',
            '/mission_complete',
            '/obstacle_markers',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_output',
            default_value='bags/drone_nav_run',
            description='Output directory for rosbag2 recording',
        ),
        LogInfo(msg=['Launching drone navigation (headless mode - no Webots simulator)']),
        LogInfo(msg=['Recording topics to rosbag2, visualizing in RViz']),
        map_publisher,
        planner,
        synthetic_pose,
        controller,
        avoidance,
        metrics,
        rviz,
        rosbag,
    ])
