from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Defaults assume the Teensy enumerates before the UWB tag. Verify
    # with `ls /dev/ttyACM*` (or dmesg) after plugging both in, and
    # override here if they land the other way around, e.g.:
    #   ros2 launch ballrobot_pkg bringup.launch.py teensy_port:=/dev/ttyACM1 uwb_port:=/dev/ttyACM0
    teensy_port_arg = DeclareLaunchArgument(
        'teensy_port', default_value='/dev/ttyACM0',
        description='Serial port for the Teensy inner-loop motor controller')
    uwb_port_arg = DeclareLaunchArgument(
        'uwb_port', default_value='/dev/ttyACM1',
        description='Serial port for the UWB tag (must differ from teensy_port)')
    imu_port_arg = DeclareLaunchArgument(
        'imu_port', default_value='/dev/serial0',
        description='Serial port for the IMU (Pi onboard UART)')

    teensy_interface_node = Node(
        package='ballrobot_pkg',
        executable='teensy_interface_node.py',
        name='teensy_interface_node',
        output='screen',
        parameters=[{'serial_port': LaunchConfiguration('teensy_port')}],
    )

    uwb_node = Node(
        package='ballrobot_pkg',
        executable='uwb_node.py',
        name='uwb_node',
        output='screen',
        parameters=[{'serial_port': LaunchConfiguration('uwb_port')}],
    )

    imu_node = Node(
        package='ballrobot_pkg',
        executable='imu_node.py',
        name='imu_node',
        output='screen',
        parameters=[{'serial_port': LaunchConfiguration('imu_port')}],
    )

    return LaunchDescription([
        teensy_port_arg,
        uwb_port_arg,
        imu_port_arg,
        teensy_interface_node,
        uwb_node,
        imu_node,
    ])
