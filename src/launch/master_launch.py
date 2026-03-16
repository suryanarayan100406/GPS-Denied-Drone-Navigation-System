import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Master launch file for GPS-Denied Drone Navigation System."""
    
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world', default_value='tunnel_world.wbt',
        description='Choose the Webots world to load'
    )
    
    # Path to Webots ROS2 package
    # NOTE: user will need to configure `webots_ros2` properly.
    # webots_ros2_dir = get_package_share_directory('webots_ros2_core')
    
    # Start Webots (Assuming basic bridge usage)
    # webots_core = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(webots_ros2_dir, 'launch', 'robot_launch.py')
    #     ),
    #     launch_arguments={'world': LaunchConfiguration('world')}.items()
    # )

    # Launch Perception Layer (EKF Fusion)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=['./src/perception_layer/config/ekf_params.yaml']
    )

    # Launch Control Layer (PID)
    # Note: executable name is typically defined in CMakeLists.txt
    # control_node = Node(
    #     package='control_layer',
    #     executable='pid_controller',
    #     name='pid_controller',
    #     output='screen'
    # )

    return LaunchDescription([
        world_arg,
        # webots_core, # Uncomment when package paths are fully built
        ekf_node,
        # control_node # Uncomment when control_layer is built
    ])
