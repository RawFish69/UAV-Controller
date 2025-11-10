from setuptools import setup
import os
from glob import glob

package_name = 'adapters_crsf'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.packers'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='CRSF adapter with pluggable packet packers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crsf_adapter_node = adapters_crsf.crsf_adapter_node:main',
            'udp_to_serial_bridge = adapters_crsf.udp_to_serial_bridge:main',
        ],
    },
)

