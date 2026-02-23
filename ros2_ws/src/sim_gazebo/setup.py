import os
from glob import glob

from setuptools import setup

package_name = 'sim_gazebo'


def collect_model_data(share_prefix: str, src_root: str = 'models'):
    data = []
    for model_dir in sorted(glob(os.path.join(src_root, '*'))):
        if not os.path.isdir(model_dir):
            continue
        top_files = sorted(
            f for f in glob(os.path.join(model_dir, '*'))
            if os.path.isfile(f)
        )
        if top_files:
            data.append((os.path.join(share_prefix, model_dir), top_files))

        for subdir in sorted(
            d for d in glob(os.path.join(model_dir, '*'))
            if os.path.isdir(d)
        ):
            sub_files = sorted(
                f for f in glob(os.path.join(subdir, '*'))
                if os.path.isfile(f)
            )
            if sub_files:
                data.append((os.path.join(share_prefix, subdir), sub_files))
    return data

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
    ] + collect_model_data(os.path.join('share', package_name), 'models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Gazebo Sim assets and bringup launches for UAV simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'terrain_marker_spawner_node = sim_gazebo.terrain_marker_spawner_node:main',
            'path_marker_spawner_node = sim_gazebo.path_marker_spawner_node:main',
            'terrain_surface_spawner_node = sim_gazebo.terrain_surface_spawner_node:main',
            'sim_reset_recovery_node = sim_gazebo.sim_reset_recovery_node:main',
        ],
    },
)
