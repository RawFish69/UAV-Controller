from setuptools import setup
import os
from glob import glob

package_name = 'px4_bridge'

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
    description='PX4 offboard bridge scaffolding (phase 2)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
    'px4_bridge_node = px4_bridge.px4_bridge_node:main',
        ],
    },
)
