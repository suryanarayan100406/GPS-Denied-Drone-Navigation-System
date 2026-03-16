import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'drone_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lenovo',
    maintainer_email='lenovo@todo.todo',
    description='Bringup package for GPS-Denied Drone',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
