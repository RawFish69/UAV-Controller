from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch manual IMU control with simulator.
    
    Starts: IMU protocol receiver, IMU controller, PID (attitude mode), 
            Safety gate, Simulator, RViz
    
    User must run IMU TX ESP32 separately (flashed with IMU_TX firmware)
    """
    
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

    sim_config = PathJoinSubstitution([
        FindPackageShare('sim_dyn'),
        'config',
        'sim_params.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('sim_dyn'),
        'rviz',
        'overview.rviz'
    ])

    # IMU protocol receiver (gets data from IMU TX ESP32)
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
            {'use_attitude_outer': True}  # Track attitude setpoints from IMU
        ]
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

    # Dynamics simulator
    sim_node = Node(
        package='sim_dyn',
        executable='dynamics_node',
        name='dynamics_node',
        output='screen',
        parameters=[sim_config]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        imu_protocol_receiver,
        imu_controller_node,
        pid_node,
        safety_node,
        sim_node,
        rviz_node
    ])

