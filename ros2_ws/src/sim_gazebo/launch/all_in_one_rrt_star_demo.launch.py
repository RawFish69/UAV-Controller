from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without GUI',
    )
    vehicle_profile_arg = DeclareLaunchArgument(
        'vehicle_profile',
        default_value='lr_drone',
        description='Vehicle profile for mission-capable sim: x3 | lr_drone',
    )
    terrain_type_arg = DeclareLaunchArgument(
        'terrain_type',
        default_value='forest',
        description='Terrain profile for terrain_generator (forest/mountains/plains)',
    )
    terrain_config_file_arg = DeclareLaunchArgument(
        'terrain_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('terrain_generator'),
            'config',
            'terrain_params.yaml',
        ]),
        description='Terrain generator YAML config (shared with sim_py for planner/terrain sync)',
    )
    start_monitor_arg = DeclareLaunchArgument(
        'start_monitor',
        default_value='true',
        description='Start ground station telemetry monitor',
    )
    start_terrain_visuals_arg = DeclareLaunchArgument(
        'start_terrain_visuals',
        default_value='true',
        description='Mirror terrain obstacle markers into Gazebo',
    )
    start_path_visuals_arg = DeclareLaunchArgument(
        'start_path_visuals',
        default_value='true',
        description='Mirror offboard planner path markers into Gazebo',
    )
    demo_delay_sec_arg = DeclareLaunchArgument(
        'demo_delay_sec',
        default_value='5.0',
        description='Delay before starting the demo mission node (seconds). '
                    'The demo itself then waits for telemetry before issuing commands, '
                    'so this just controls when the node is launched.',
    )
    planner_type_arg = DeclareLaunchArgument(
        'planner_type',
        default_value='rrt*',
        description='Planner type requested by demo mission (e.g. astar, rrt, rrt*)',
    )
    planner_collision_inflation_m_arg = DeclareLaunchArgument(
        'planner_collision_inflation_m',
        default_value='2.5',
        description='Collision inflation passed to offboard planner demo requests (higher = paths avoid obstacles)',
    )
    planning_mode_arg = DeclareLaunchArgument(
        'planning_mode',
        default_value='onboard',
        description='Demo mission planning mode: onboard (air unit plans) | offboard (ground plans)',
    )
    path_marker_topic_arg = DeclareLaunchArgument(
        'path_marker_topic',
        default_value='/uav/planner/planned_path_markers',
        description='Path marker topic for Gazebo visuals (onboard: /uav/planner/..., offboard: /gs/planner/...)',
    )
    allow_handcrafted_fallback_arg = DeclareLaunchArgument(
        'allow_handcrafted_fallback',
        default_value='true',
        description='Allow a hardcoded fallback mission if all planner attempts fail (ensures drone can take off)',
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Start rviz2 for debugging (markers/telemetry)',
    )

    sim_gazebo_share = FindPackageShare('sim_gazebo')
    planner_safe_params = PathJoinSubstitution([sim_gazebo_share, 'config', 'planner_server_safe_rrtstar.yaml'])
    mission_executor_safe_params = PathJoinSubstitution(
        [sim_gazebo_share, 'config', 'mission_executor_safe_tracking.yaml']
    )
    command_manager_safe_params = PathJoinSubstitution(
        [sim_gazebo_share, 'config', 'command_manager_safe_custom.yaml']
    )

    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sim_gazebo'),
                'launch',
                'bringup.launch.py',
            ])
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'run_smoke_cmd': 'false',
            'start_air_unit': 'true',
            'start_air_planner': 'true',  # air unit planner runs so onboard planning works and path markers show
            'start_terrain_visuals': LaunchConfiguration('start_terrain_visuals'),
            'start_path_visuals': LaunchConfiguration('start_path_visuals'),
            'path_marker_topic': LaunchConfiguration('path_marker_topic'),
            # Pass safe params files to both air unit nodes
            'use_mission_executor_params_file': 'true',
            'mission_executor_params_file': mission_executor_safe_params,
            'use_command_manager_params_file': 'true',
            'command_manager_params_file': command_manager_safe_params,
        }.items(),
    )

    terrain_gen = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('terrain_generator'),
                'launch',
                'terrain_generator.launch.py',
            ])
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('terrain_config_file'),
            'terrain_type': LaunchConfiguration('terrain_type'),
        }.items(),
    )

    ground_station_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ground_station'),
                'launch',
                'ground.launch.py',
            ])
        ),
        launch_arguments={
            'start_monitor': LaunchConfiguration('start_monitor'),
            'start_demo': 'false',
            'start_planner': 'false',  # onboard default: air unit has planner; set true for offboard
            'use_planner_params_file': 'true',
            'planner_params_file': planner_safe_params,
        }.items(),
    )

    demo_mission = Node(
        package='ground_station',
        executable='ground_station_demo_mission',
        name='ground_station_demo_mission',
        output='screen',
        parameters=[{
            'planning_mode': LaunchConfiguration('planning_mode'),
            'use_planner': True,
            'planner_type': LaunchConfiguration('planner_type'),
            'terrain_profile': LaunchConfiguration('terrain_type'),
            'planner_collision_inflation_m': LaunchConfiguration('planner_collision_inflation_m'),
            'planner_failure_policy': 'fallback_planner',
            'planner_fallback_order': ['rrt', 'astar'],
            'allow_handcrafted_fallback': LaunchConfiguration('allow_handcrafted_fallback'),
            'planner_call_timeout_sec': 25.0,
            # Start near one edge and fly toward the far side (SimPy-like behavior).
            'mission_goal_xyz': [180.0, 140.0, 3.0],
            'mission_via1_xyz': [55.0, 40.0, 3.0],
            'mission_via2_xyz': [110.0, 85.0, 3.0],
            # Wait for bridge/drone to be up before starting (replaces fixed timer approach)
            'wait_for_telemetry_sec': 45.0,
            # Do not block takeoff on terrain; drone arms and takes off as soon as trajectory is ready.
            'terrain_marker_topic': '/terrain/obstacles',
            'wait_for_terrain_sec': 8.0,
            'min_terrain_markers': 0,
            'path_marker_topic': LaunchConfiguration('path_marker_topic'),
            'wait_for_path_markers_sec': 15.0,
            'min_path_markers': 0,
            'require_path_markers_before_takeoff': False,
            # Cap planner wait so we take off quickly; after this we use handcrafted and arm.
            'planner_call_timeout_sec': 10.0,
            'max_total_planning_sec': 12.0,
            'takeoff_wait_sec': 6.0,
            # Stable hover in the air before mission: give the drone time to settle
            'hover_wait_sec': 14.0,
            'pre_mission_hover_sec': 6.0,
            'mission_timeout_sec': 300.0,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    delayed_demo_mission = TimerAction(
        period=LaunchConfiguration('demo_delay_sec'),
        actions=[demo_mission],
    )

    return LaunchDescription([
        headless_arg,
        vehicle_profile_arg,
        terrain_type_arg,
        terrain_config_file_arg,
        start_monitor_arg,
        start_terrain_visuals_arg,
        start_path_visuals_arg,
        demo_delay_sec_arg,
        planner_type_arg,
        planner_collision_inflation_m_arg,
        planning_mode_arg,
        path_marker_topic_arg,
        allow_handcrafted_fallback_arg,
        start_rviz_arg,
        sim_bringup,
        terrain_gen,
        ground_station_stack,
        delayed_demo_mission,
        rviz_node,
    ])
