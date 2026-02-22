from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='sim_gazebo bringup placeholder - to be implemented'),
    ])
