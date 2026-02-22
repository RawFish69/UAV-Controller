from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_ground_arg = DeclareLaunchArgument('start_ground', default_value='true')
    start_demo_arg = DeclareLaunchArgument('start_demo', default_value='false')
    start_monitor_arg = DeclareLaunchArgument('start_monitor', default_value='true')
    start_offboard_planner_arg = DeclareLaunchArgument('start_offboard_planner', default_value='true')
    start_onboard_planner_arg = DeclareLaunchArgument('start_onboard_planner', default_value='false')
    demo_planning_mode_arg = DeclareLaunchArgument('demo_planning_mode', default_value='offboard')

    return LaunchDescription([
        start_ground_arg,
        start_demo_arg,
        start_monitor_arg,
        start_offboard_planner_arg,
        start_onboard_planner_arg,
        demo_planning_mode_arg,

        Node(
            package='sim_bridge',
            executable='fastsim_backend_adapter_node',
            name='fastsim_backend_adapter_node',
            output='screen',
        ),
        Node(
            package='air_unit',
            executable='telemetry_adapter_node',
            name='telemetry_adapter_node',
            output='screen',
        ),
        Node(
            package='air_unit',
            executable='mission_executor_node',
            name='mission_executor_node',
            output='screen',
        ),
        Node(
            package='air_unit',
            executable='command_manager_node',
            name='command_manager_node',
            output='screen',
        ),
        Node(
            package='planner',
            executable='planner_server_node',
            namespace='uav1/planner',
            name='planner_server_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_onboard_planner')),
        ),
        Node(
            package='planner',
            executable='planner_server_node',
            namespace='gs/planner',
            name='planner_server_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_offboard_planner')),
        ),
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
            parameters=[{
                'planning_mode': LaunchConfiguration('demo_planning_mode'),
            }],
        ),
    ])
