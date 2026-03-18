"""
Launch drone navigation with Webots GUI simulator and video recording using ffmpeg.
Records the Webots display output to MP4 video file.
"""

import os
import subprocess
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher


def start_screen_recording(context, *args, **kwargs):
    """Start recording the screen with ffmpeg."""
    # Try to record the first active display
    output_file = context.launch_configurations.get('video_output', 'videos/drone_nav_demo.mp4')
    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)
    
    # ffmpeg command to record desktop at 30 fps
    record_cmd = [
        'ffmpeg',
        '-f', 'gdigrab',
        '-framerate', '30',
        '-i', 'desktop',  # Windows desktop capture
        '-c:v', 'libx264',
        '-crf', '23',
        '-preset', 'fast',
        output_file,
    ]
    
    print(f"\nStarting video recording to {output_file}")
    return [
        ExecuteProcess(
            cmd=record_cmd,
            output='screen',
            name='ffmpeg_recorder',
        )
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('drone_nav_2d')
    params_file = os.path.join(pkg_share, 'config', 'nav_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'drone_nav.rviz')
    world_easy = os.path.join(pkg_share, 'worlds', 'drone_world.wbt')
    world_hard = os.path.join(pkg_share, 'worlds', 'drone_world_hard.wbt')
    urdf_path = os.path.join(pkg_share, 'urdf', 'drone.urdf')

    use_hard_world = LaunchConfiguration('use_hard_world')
    bag_output = LaunchConfiguration('bag_output')
    video_output = LaunchConfiguration('video_output')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    # Select world based on argument
    selected_world = world_hard if use_hard_world else world_easy

    webots = WebotsLauncher(
        world=selected_world,
        ros2_supervisor=True,
    )

    bridge_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={
            'WEBOTS_CONTROLLER_URL': 'drone',
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

    map_publisher = Node(
        package='drone_nav_2d',
        executable='map_publisher',
        name='map_publisher',
        output='screen',
        parameters=[params_file],
    )

    planner = Node(
        package='drone_nav_2d',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[params_file],
    )

    controller = Node(
        package='drone_nav_2d',
        executable='drone_controller',
        name='drone_controller',
        output='screen',
        parameters=[params_file],
    )

    avoidance = Node(
        package='drone_nav_2d',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[params_file],
    )

    metrics = Node(
        package='drone_nav_2d',
        executable='metrics_logger',
        name='metrics_logger',
        output='screen',
        parameters=[params_file],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

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
            'use_hard_world',
            default_value='false',
            description='Set true to launch drone_world_hard.wbt (maze)',
        ),
        DeclareLaunchArgument(
            'bag_output',
            default_value='bags/drone_nav_run',
            description='Output directory for rosbag2 recording',
        ),
        DeclareLaunchArgument(
            'video_output',
            default_value='videos/drone_nav_demo.mp4',
            description='Output file for video recording',
        ),
        LogInfo(msg=['Launching Webots drone simulator with video recording']),
        webots,
        bridge_driver,
        map_publisher,
        planner,
        controller,
        avoidance,
        metrics,
        rviz,
        rosbag,
        OpaqueFunction(function=start_screen_recording),
    ])
