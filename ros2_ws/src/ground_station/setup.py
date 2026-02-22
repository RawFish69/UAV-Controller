from setuptools import setup
import os
from glob import glob

package_name = 'ground_station'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
('share/ament_index/resource_index/packages', ['resource/' + package_name]),
('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
(os.path.join('share', package_name, 'config'), glob('config/*')),
(os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
(os.path.join('share', package_name, 'models', 'x3_velocity_control'), glob('models/x3_velocity_control/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Ground station ROS2 nodes for commands, missions, and telemetry',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
    'ground_station_cli = ground_station.ground_station_cli:main',
    'ground_station_telemetry_monitor = ground_station.ground_station_telemetry_monitor:main',
    'ground_station_demo_mission = ground_station.ground_station_demo_mission:main',
        ],
    },
)
