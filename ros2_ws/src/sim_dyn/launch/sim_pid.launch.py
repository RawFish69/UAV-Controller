from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Config files
    sim_config = PathJoinSubstitution([
        FindPackageShare('sim_dyn'),
        'config',
        'sim_params.yaml'
    ])

    pid_config = PathJoinSubstitution([
        FindPackageShare('controllers_pid'),
        'config',
        'pid_params.yaml'
    ])

    safety_config = PathJoinSubstitution([
        FindPackageShare('safety_gate'),
        'config',
        'safety_params.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('sim_dyn'),
        'rviz',
        'overview.rviz'
    ])

    # Dynamics simulator
    sim_node = Node(
        package='sim_dyn',
        executable='dynamics_node',
        name='dynamics_node',
        output='screen',
        parameters=[sim_config]
    )

    # PID controller
    pid_node = Node(
        package='controllers_pid',
        executable='pid_node',
        name='pid_controller',
        output='screen',
        parameters=[pid_config]
    )

    # Safety gate (sim mode)
    safety_node = Node(
        package='safety_gate',
        executable='safety_gate_node',
        name='safety_gate',
        output='screen',
        parameters=[
            safety_config,
            {'mode': 'sim'}
        ]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        sim_node,
        pid_node,
        safety_node,
        rviz_node
    ])

