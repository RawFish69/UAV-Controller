from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_urdf = PathJoinSubstitution([
        FindPackageShare('sim_gazebo'),
        'models',
        'lr_drone_urdf',
        'lr_drone_urdf.urdf',
    ])
    default_rviz = PathJoinSubstitution([
        FindPackageShare('sim_gazebo'),
        'config',
        'lr_drone_urdf.rviz',
    ])

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf,
        description='URDF file to visualize in RViz',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='RViz config file',
    )

    robot_description = ParameterValue(
        Command(['cat ', LaunchConfiguration('urdf_file')]),
        value_type=str,
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        urdf_file_arg,
        rviz_config_arg,
        rsp,
        jsp,
        rviz,
    ])
