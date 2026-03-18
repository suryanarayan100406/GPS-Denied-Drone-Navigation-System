from setuptools import setup

package_name = 'drone_nav_2d'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/drone_nav_launch.py']),
        (f'share/{package_name}/config', ['config/nav_params.yaml']),
        (f'share/{package_name}/rviz', ['rviz/drone_nav.rviz']),
        (f'share/{package_name}/worlds', ['worlds/drone_world.wbt', 'worlds/drone_world_hard.wbt']),
        (f'share/{package_name}/controllers/moving_wall_controller', ['controllers/moving_wall_controller/moving_wall_controller.py']),
        (f'share/{package_name}/urdf', ['urdf/drone.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robothon Team',
    maintainer_email='team@example.com',
    description='Autonomous 2D drone navigation in Webots using ROS2 Humble.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_publisher = drone_nav_2d.map_publisher:main',
            'path_planner = drone_nav_2d.path_planner:main',
            'drone_controller = drone_nav_2d.drone_controller:main',
            'obstacle_avoidance = drone_nav_2d.obstacle_avoidance:main',
            'metrics_logger = drone_nav_2d.metrics_logger:main',
        ],
    },
)
