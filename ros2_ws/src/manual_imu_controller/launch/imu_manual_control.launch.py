from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Config files
    imu_protocol_config = PathJoinSubstitution([
        FindPackageShare('manual_imu_controller'),
        'config',
        'imu_protocol_params.yaml'
    ])
    
    imu_config = PathJoinSubstitution([
        FindPackageShare('manual_imu_controller'),
        'config',
        'imu_controller_params.yaml'
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

    # Declare arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Output mode: sim or crsf'
    )

    # IMU protocol receiver
    imu_protocol_receiver = Node(
        package='manual_imu_controller',
        executable='imu_protocol_receiver',
        name='imu_protocol_receiver',
        output='screen',
        parameters=[imu_protocol_config]
    )

    # IMU gesture controller
    imu_controller_node = Node(
        package='manual_imu_controller',
        executable='imu_controller_node',
        name='imu_controller_node',
        output='screen',
        parameters=[imu_config]
    )

    # PID controller (with attitude outer loop enabled)
    pid_node = Node(
        package='controllers_pid',
        executable='pid_node',
        name='pid_controller',
        output='screen',
        parameters=[
            pid_config,
            {'use_attitude_outer': True}  # Enable attitude tracking
        ]
    )

    # Safety gate
    safety_node = Node(
        package='safety_gate',
        executable='safety_gate_node',
        name='safety_gate',
        output='screen',
        parameters=[
            safety_config,
            {'mode': LaunchConfiguration('mode')}
        ]
    )

    return LaunchDescription([
        mode_arg,
        imu_protocol_receiver,
        imu_controller_node,
        pid_node,
        safety_node
    ])

