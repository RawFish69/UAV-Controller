from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without GUI',
    )
    start_terrain_visuals_arg = DeclareLaunchArgument(
        'start_terrain_visuals',
        default_value='false',
        description='Mirror terrain markers into Gazebo',
    )
    start_path_visuals_arg = DeclareLaunchArgument(
        'start_path_visuals',
        default_value='false',
        description='Mirror planner path markers into Gazebo',
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sim_gazebo'),
                'launch',
                'bringup.launch.py',
            ])
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'vehicle_profile': 'lr_drone',
            'start_air_unit': 'true',
            'start_air_planner': 'false',
            'start_terrain_visuals': LaunchConfiguration('start_terrain_visuals'),
            'start_path_visuals': LaunchConfiguration('start_path_visuals'),
            'run_smoke_cmd': 'false',
        }.items(),
    )

    return LaunchDescription([
        headless_arg,
        start_terrain_visuals_arg,
        start_path_visuals_arg,
        bringup,
    ])
