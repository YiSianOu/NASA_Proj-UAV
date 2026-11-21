"""Start the controller and terminal monitor; run the keyboard separately."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyUSB0',
            description='PX4 MAVLink serial device or pymavlink connection URL',
        ),
        DeclareLaunchArgument(
            'baud', default_value='57600',
            description='Serial baud rate; must match the PX4 port',
        ),
        Node(
            package='keyboard_flight',
            executable='controller',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int),
            }],
        ),
        Node(
            package='keyboard_flight',
            executable='monitor',
            output='screen',
            emulate_tty=True,
        ),
    ])
