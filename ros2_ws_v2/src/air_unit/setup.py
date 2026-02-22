from setuptools import setup
import os
from glob import glob

package_name = 'air_unit'

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
    description='Air unit ROS2 nodes for command management, mission execution, and telemetry',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
    'command_manager_node = air_unit.command_manager_node:main',
    'telemetry_adapter_node = air_unit.telemetry_adapter_node:main',
    'mission_executor_node = air_unit.mission_executor_node:main',
        ],
    },
)
