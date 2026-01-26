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
            FindPackageShare('terrain_generator'),
            'config',
            'terrain_params.yaml'
        ]),
        description='Path to terrain generator config file'
    )
    
    terrain_type_arg = DeclareLaunchArgument(
        'terrain_type',
        default_value='forest',
        description='Terrain type: forest, mountains, or plains'
    )

    # Terrain generator node
    terrain_node = Node(
        package='terrain_generator',
        executable='terrain_generator_node',
        name='terrain_generator',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'terrain_type': LaunchConfiguration('terrain_type')}
        ]
    )

    return LaunchDescription([
        config_file_arg,
        terrain_type_arg,
        terrain_node
    ])
