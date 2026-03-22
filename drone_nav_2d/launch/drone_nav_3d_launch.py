import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_unique_bag_output(bag_output_base: str) -> str:
    """Create unique rosbag output path by appending timestamp if folder exists."""
    candidate = bag_output_base
    if os.path.exists(candidate):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        candidate = f'{bag_output_base}_3d_{timestamp}'
    return candidate


def _build_actions(context):
    """Build launch actions dynamically based on configuration."""
    pkg_share = get_package_share_directory('drone_nav_2d')
    params_file = os.path.join(pkg_share, 'config', 'nav_params_3d.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'drone_nav.rviz')
    urdf_path = os.path.join(pkg_share, 'urdf', 'drone_3d.urdf')

    # Get world type from launch argument
    world_type = LaunchConfiguration('world_type').perform(context).strip().lower()
    bag_output_base = LaunchConfiguration('bag_output').perform(context).strip()
    bag_output = _resolve_unique_bag_output(bag_output_base)

    # Select world file based on type
    world_map = {
        'urban': os.path.join(pkg_share, 'worlds', 'drone_world_urban_3d.wbt'),
        'forest': os.path.join(pkg_share, 'worlds', 'drone_world_forest_3d.wbt'),
        'warehouse': os.path.join(pkg_share, 'worlds', 'drone_world_warehouse_3d.wbt'),
        'mixed': os.path.join(pkg_share, 'worlds', 'drone_world_mixed_3d.wbt'),
    }
    selected_world = world_map.get(world_type, world_map['urban'])

    # Read URDF for robot description
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    # Get Webots path from environment
    webots_home = os.environ.get('WEBOTS_HOME', '')
    webots_executable = os.path.join(webots_home, 'webots') if webots_home else 'webots'

    # Webots simulator (headless or GUI)
    webots = ExecuteProcess(
        cmd=[
            webots_executable,
            '--port=1234',
            selected_world,
            '--batch',
            '--mode=realtime',
        ],
        output='screen',
        name='webots_3d',
    )

    # Webots ROS2 driver bridge
    bridge_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={
            'WEBOTS_CONTROLLER_URL': 'tcp://127.0.0.1:1234/drone',
        },
        parameters=[
            {'robot_description': robot_description},
        ],
        remappings=[
            ('/gps', '/webots/drone/gps'),
            ('/scan', '/webots/drone/scan'),
            ('/cmd_vel', '/cmd_vel'),
        ],
    )

    # 3D Map Publisher (voxel grid visualization)
    map_publisher = Node(
        package='drone_nav_2d',
        executable='map_publisher_3d',
        name='map_publisher_3d',
        output='screen',
        parameters=[params_file],
    )

    # 3D Path Planner (A* + RRT)
    planner = Node(
        package='drone_nav_2d',
        executable='path_planner_3d',
        name='path_planner_3d',
        output='screen',
        parameters=[params_file],
    )

    # 6-DoF Drone Controller
    controller = Node(
        package='drone_nav_2d',
        executable='drone_controller_3d',
        name='drone_controller_3d',
        output='screen',
        parameters=[params_file],
    )

    # 3D Obstacle Avoidance
    avoidance = Node(
        package='drone_nav_2d',
        executable='obstacle_avoidance_3d',
        name='obstacle_avoidance_3d',
        output='screen',
        parameters=[params_file],
    )

    # Metrics Logger
    metrics = Node(
        package='drone_nav_2d',
        executable='metrics_logger',
        name='metrics_logger_3d',
        output='screen',
        parameters=[params_file],
    )

    # RViz2 visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_3d',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # Rosbag recording
    rosbag = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'record',
            '-o',
            bag_output,
            '/map',
            '/planned_path_3d',
            '/drone_pose_3d',
            '/drone_trajectory_3d',
            '/cmd_vel_3d',
            '/obstacle_detected_3d',
            '/replan_event_3d',
            '/min_obstacle_distance_3d',
            '/mission_complete_3d',
            '/obstacles_3d',
        ],
        output='screen',
    )

    actions = [
        LogInfo(msg=[f'Launching 3D drone navigation: {world_type} scenario']),
        LogInfo(msg=[f'Rosbag output: {bag_output}']),
        webots,
        bridge_driver,
        map_publisher,
        planner,
        controller,
        avoidance,
        metrics,
        rviz,
        rosbag,
    ]

    return actions


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for 3D drone navigation system."""
    actions = [
        DeclareLaunchArgument(
            'world_type',
            default_value='urban',
            description='3D world scenario: urban | forest | warehouse | mixed',
        ),
        DeclareLaunchArgument(
            'bag_output',
            default_value='bags/drone_nav_3d_run',
            description='Output directory for rosbag2 recording',
        ),
        OpaqueFunction(function=_build_actions),
    ]

    return LaunchDescription(actions)
