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
            FindPackageShare('controllers_mpc'),
            'config',
            'mpc_params.yaml'
        ]),
        description='Path to MPC config file'
    )

    # MPC controller node
    mpc_node = Node(
        package='controllers_mpc',
        executable='mpc_node',
        name='mpc_controller',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_file_arg,
        mpc_node
    ])
