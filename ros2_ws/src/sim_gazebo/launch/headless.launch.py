from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='flat_world_lr_drone_preview.sdf',
        description='World SDF file in sim_gazebo/worlds',
    )
    bridge_config_file_arg = DeclareLaunchArgument(
        'bridge_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sim_gazebo'),
            'config',
            'bridge_topics_preview.yaml',
        ]),
        description='ros_gz_bridge YAML config file path',
    )
    gz_vehicle_namespace_arg = DeclareLaunchArgument(
        'gz_vehicle_namespace',
        default_value='lr_drone',
        description='Gazebo vehicle namespace used in command/enable topics',
    )
    gz_model_name_arg = DeclareLaunchArgument(
        'gz_model_name',
        default_value='lr_drone',
        description='Gazebo model entity name used for odometry topic',
    )

    return LaunchDescription([
        world_file_arg,
        bridge_config_file_arg,
        gz_vehicle_namespace_arg,
        gz_model_name_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('sim_gazebo'),
                    'launch',
                    'bringup.launch.py',
                ])
            ),
            launch_arguments={
                'headless': 'true',
                'start_paused': 'true',
                'start_bridge': 'false',
                'start_gazebo_adapter': 'false',
                'run_smoke_cmd': 'false',
                'start_air_unit': 'false',
                'start_air_planner': 'false',
                'world_file': LaunchConfiguration('world_file'),
                'bridge_config_file': LaunchConfiguration('bridge_config_file'),
                'gz_vehicle_namespace': LaunchConfiguration('gz_vehicle_namespace'),
                'gz_model_name': LaunchConfiguration('gz_model_name'),
            }.items(),
        )
    ])
