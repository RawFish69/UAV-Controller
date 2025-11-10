from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('controllers_pid'),
            'config',
            'pid_params.yaml'
        ]),
        description='Path to PID config file'
    )

    # PID controller node
    pid_node = Node(
        package='controllers_pid',
        executable='pid_node',
        name='pid_controller',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_file_arg,
        pid_node
    ])

