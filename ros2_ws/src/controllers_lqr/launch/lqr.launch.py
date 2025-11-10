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
            FindPackageShare('controllers_lqr'),
            'config',
            'lqr_params.yaml'
        ]),
        description='Path to LQR config file'
    )

    # LQR controller node
    lqr_node = Node(
        package='controllers_lqr',
        executable='lqr_node',
        name='lqr_controller',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_file_arg,
        lqr_node
    ])

