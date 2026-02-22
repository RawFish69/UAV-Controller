from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ground_station',
            executable='ground_station_telemetry_monitor',
            name='ground_station_telemetry_monitor',
            output='screen',
        ),
    ])
