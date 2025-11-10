from setuptools import setup
import os
from glob import glob

package_name = 'sim_dyn'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='UAV dynamics simulator with RViz visualization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamics_node = sim_dyn.dynamics_node:main',
        ],
    },
)

