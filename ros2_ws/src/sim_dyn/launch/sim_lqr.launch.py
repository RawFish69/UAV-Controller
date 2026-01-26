from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 for visualization (set false for headless)',
    )

    # Config files
    sim_config = PathJoinSubstitution([
        FindPackageShare('sim_dyn'),
        'config',
        'sim_params.yaml'
    ])

    lqr_config = PathJoinSubstitution([
        FindPackageShare('controllers_lqr'),
        'config',
        'lqr_params.yaml'
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

    # LQR controller
    lqr_node = Node(
        package='controllers_lqr',
        executable='lqr_node',
        name='lqr_controller',
        output='screen',
        parameters=[lqr_config]
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

    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_rviz_arg,
        sim_node,
        lqr_node,
        safety_node,
        rviz_node,
    ])

