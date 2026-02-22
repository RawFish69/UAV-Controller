from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    monitor_arg = DeclareLaunchArgument('start_monitor', default_value='true')
    demo_arg = DeclareLaunchArgument('start_demo', default_value='false')
    planner_arg = DeclareLaunchArgument('start_planner', default_value='false')

    return LaunchDescription([
        monitor_arg,
        demo_arg,
        planner_arg,
        Node(
            package='ground_station',
            executable='ground_station_telemetry_monitor',
            name='ground_station_telemetry_monitor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_monitor')),
        ),
        Node(
            package='ground_station',
            executable='ground_station_demo_mission',
            name='ground_station_demo_mission',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_demo')),
        ),
        Node(
            package='planner',
            executable='planner_server_node',
            name='planner_server_node',
            namespace='gs/planner',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_planner')),
        ),
    ])
