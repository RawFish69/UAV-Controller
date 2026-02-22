from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare('sim_gazebo'),
        'worlds',
        'flat_world.sdf',
    ])
    bridge_config = PathJoinSubstitution([
        FindPackageShare('sim_gazebo'),
        'config',
        'bridge_topics.yaml',
    ])

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without GUI',
    )
    run_smoke_arg = DeclareLaunchArgument(
        'run_smoke_cmd',
        default_value='false',
        description='Start a simple smoke command publisher node',
    )
    start_air_arg = DeclareLaunchArgument(
        'start_air_unit',
        default_value='true',
        description='Start air unit nodes',
    )
    start_air_planner_arg = DeclareLaunchArgument(
        'start_air_planner',
        default_value='true',
        description='Start air-side planner service (/uav/planner/plan_path)',
    )
    start_terrain_visuals_arg = DeclareLaunchArgument(
        'start_terrain_visuals',
        default_value='false',
        description='Spawn /terrain/obstacles markers as Gazebo entities',
    )
    terrain_world_name_arg = DeclareLaunchArgument(
        'terrain_world_name',
        default_value='uav_flat_world',
        description='Gazebo world name for terrain spawning service path',
    )
    start_terrain_surface_arg = DeclareLaunchArgument(
        'start_terrain_surface',
        default_value='false',
        description='Spawn a generated terrain surface mesh into Gazebo (mountains profile)',
    )
    start_path_visuals_arg = DeclareLaunchArgument(
        'start_path_visuals',
        default_value='false',
        description='Spawn planner waypoint markers (spheres) in Gazebo from planner MarkerArray topic',
    )
    path_marker_topic_arg = DeclareLaunchArgument(
        'path_marker_topic',
        default_value='/gs/planner/planned_path_markers',
        description='Planner MarkerArray topic to mirror into Gazebo',
    )
    terrain_surface_type_arg = DeclareLaunchArgument(
        'terrain_surface_type',
        default_value='mountains',
        description='Terrain profile for surface mesh generator (expects mountains)',
    )
    terrain_config_file_arg = DeclareLaunchArgument(
        'terrain_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('terrain_generator'),
            'config',
            'terrain_params.yaml',
        ]),
        description='Shared terrain config YAML used for terrain surface generation',
    )

    gz_cmd = [
        'gz', 'sim',
        '-r',
        world,
    ]
    gz_cmd_headless = [
        'gz', 'sim',
        '-r',
        '-s',
        world,
    ]

    gz_sim_gui = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )
    gz_sim_headless = ExecuteProcess(
        cmd=gz_cmd_headless,
        output='screen',
        condition=IfCondition(LaunchConfiguration('headless')),
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    gazebo_adapter = Node(
        package='sim_bridge',
        executable='gazebo_backend_adapter_node',
        name='gazebo_backend_adapter',
        output='screen',
        parameters=[{
            'uav_namespace': '/uav',
            'backend_cmd_topic': '/uav/backend/cmd_twist',
            'backend_enable_topic': '/uav/backend/enable',
            'backend_odom_topic': '/uav/backend/odom',
            'gz_cmd_topic': '/X3/gazebo/command/twist',
            'gz_enable_topic': '/X3/enable',
            'gz_odom_topic': '/model/x3/odometry',
        }],
    )

    smoke_node = Node(
        package='sim_bridge',
        executable='gz_smoke_cmd_node',
        name='gz_smoke_cmd_node',
        output='screen',
        parameters=[{
            'cmd_topic': '/uav/backend/cmd_twist',
            'enable_topic': '/uav/backend/enable',
        }],
        condition=IfCondition(LaunchConfiguration('run_smoke_cmd')),
    )

    terrain_visual_spawner = Node(
        package='sim_gazebo',
        executable='terrain_marker_spawner_node',
        name='terrain_marker_spawner',
        output='screen',
        parameters=[{
            'marker_topic': '/terrain/obstacles',
            'world_name': LaunchConfiguration('terrain_world_name'),
            'spawn_once': True,
        }],
        condition=IfCondition(LaunchConfiguration('start_terrain_visuals')),
    )

    terrain_surface_spawner = Node(
        package='sim_gazebo',
        executable='terrain_surface_spawner_node',
        name='terrain_surface_spawner',
        output='screen',
        parameters=[{
            'world_name': LaunchConfiguration('terrain_world_name'),
            'terrain_type': LaunchConfiguration('terrain_surface_type'),
            'terrain_config_file': LaunchConfiguration('terrain_config_file'),
        }],
        condition=IfCondition(LaunchConfiguration('start_terrain_surface')),
    )

    path_visual_spawner = Node(
        package='sim_gazebo',
        executable='terrain_marker_spawner_node',
        name='path_marker_spawner',
        output='screen',
        parameters=[{
            'marker_topic': LaunchConfiguration('path_marker_topic'),
            'world_name': LaunchConfiguration('terrain_world_name'),
            'model_name_prefix': 'planned_path',
            'spawn_once': True,
            'create_collision': False,
        }],
        condition=IfCondition(LaunchConfiguration('start_path_visuals')),
    )

    air_nodes = [
        Node(
            package='air_unit',
            executable='telemetry_adapter_node',
            name='telemetry_adapter_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_air_unit')),
        ),
        Node(
            package='air_unit',
            executable='mission_executor_node',
            name='mission_executor_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_air_unit')),
        ),
        Node(
            package='air_unit',
            executable='command_manager_node',
            name='command_manager_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_air_unit')),
        ),
        Node(
            package='planner',
            executable='planner_server_node',
            name='planner_server_node',
            namespace='uav/planner',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_air_planner')),
        ),
    ]

    return LaunchDescription([
        headless_arg,
        run_smoke_arg,
        start_air_arg,
        start_air_planner_arg,
        start_terrain_visuals_arg,
        terrain_world_name_arg,
        start_terrain_surface_arg,
        start_path_visuals_arg,
        path_marker_topic_arg,
        terrain_surface_type_arg,
        terrain_config_file_arg,
        gz_sim_gui,
        gz_sim_headless,
        bridge_node,
        gazebo_adapter,
        smoke_node,
        terrain_visual_spawner,
        terrain_surface_spawner,
        path_visual_spawner,
        *air_nodes,
    ])
