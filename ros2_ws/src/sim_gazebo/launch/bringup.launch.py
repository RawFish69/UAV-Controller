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
        gz_sim_gui,
        gz_sim_headless,
        bridge_node,
        gazebo_adapter,
        smoke_node,
        *air_nodes,
    ])
