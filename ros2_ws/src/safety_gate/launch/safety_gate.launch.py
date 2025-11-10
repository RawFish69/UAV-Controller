from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Operating mode: sim or crsf'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('safety_gate'),
            'config',
            'safety_params.yaml'
        ]),
        description='Path to safety gate config file'
    )

    # Safety gate node
    safety_gate_node = Node(
        package='safety_gate',
        executable='safety_gate_node',
        name='safety_gate',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'mode': LaunchConfiguration('mode')}
        ]
    )

    return LaunchDescription([
        mode_arg,
        config_file_arg,
        safety_gate_node
    ])

