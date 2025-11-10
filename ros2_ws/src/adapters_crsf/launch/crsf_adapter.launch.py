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
            FindPackageShare('adapters_crsf'),
            'config',
            'crsf_params.yaml'
        ]),
        description='Path to CRSF adapter config file'
    )

    transport_arg = DeclareLaunchArgument(
        'transport',
        default_value='udp',
        description='Transport mode: udp or serial'
    )

    udp_host_arg = DeclareLaunchArgument(
        'udp_host',
        default_value='192.168.4.1',
        description='UDP host address'
    )

    udp_port_arg = DeclareLaunchArgument(
        'udp_port',
        default_value='9000',
        description='UDP port'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port device'
    )

    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='921600',
        description='Serial baud rate'
    )

    packer_arg = DeclareLaunchArgument(
        'packer',
        default_value='txrx_packer',
        description='Packet packer module name'
    )

    # CRSF adapter node
    crsf_adapter_node = Node(
        package='adapters_crsf',
        executable='crsf_adapter_node',
        name='crsf_adapter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'transport': LaunchConfiguration('transport'),
                'udp_host': LaunchConfiguration('udp_host'),
                'udp_port': LaunchConfiguration('udp_port'),
                'serial_port': LaunchConfiguration('serial_port'),
                'baud': LaunchConfiguration('baud'),
                'packer': LaunchConfiguration('packer'),
            }
        ]
    )

    return LaunchDescription([
        config_file_arg,
        transport_arg,
        udp_host_arg,
        udp_port_arg,
        serial_port_arg,
        baud_arg,
        packer_arg,
        crsf_adapter_node
    ])

