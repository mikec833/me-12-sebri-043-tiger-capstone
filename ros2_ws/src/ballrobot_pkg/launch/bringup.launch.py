from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
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

    # imu_quaternion_node is a debug/visualization aid (converts imu/data's
    # yaw/pitch/roll to a quaternion on imu/data_quat for RViz/rqt_plot/
    # topic echo) rather than something the robot needs every run, so it's
    # opt-in:
    #   ros2 launch ballrobot_pkg bringup.launch.py enable_imu_quat:=true
    enable_imu_quat_arg = DeclareLaunchArgument(
        'enable_imu_quat', default_value='false',
        description='Also start imu_quaternion_node.py (debug/visualization aid)')

    # Computed once per `ros2 launch` invocation (generate_launch_description()
    # runs once), then handed to every node below as the `run_id` parameter,
    # so their run-numbered CSVs (imu_log_run<id>.csv, uwb_log_run<id>.csv,
    # wheel_speed_log_run<id>.csv, cmd_vel_log_run<id>.csv, ...) all share
    # the same <id> -- see run_logging.py. If you separately start a node
    # this launch doesn't cover (e.g. imu_quaternion_node.py with
    # enable_imu_quat left false), pass the same value with
    # `--ros-args -p run_id:=<id>` to keep its CSV in sync too; the LogInfo
    # below prints it for that purpose. Override run_id here to reuse a
    # specific id instead, e.g. to resume logging into an existing run.
    run_id_arg = DeclareLaunchArgument(
        'run_id', default_value=datetime.now().strftime('%Y%m%d_%H%M%S'),
        description='Shared CSV run identifier for every node this launch starts')

    teensy_interface_node = Node(
        package='ballrobot_pkg',
        executable='teensy_interface_node.py',
        name='teensy_interface_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('teensy_port'),
            'run_id': LaunchConfiguration('run_id'),
        }],
    )

    uwb_node = Node(
        package='ballrobot_pkg',
        executable='uwb_node.py',
        name='uwb_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('uwb_port'),
            'run_id': LaunchConfiguration('run_id'),
        }],
    )

    imu_node = Node(
        package='ballrobot_pkg',
        executable='imu_node.py',
        name='imu_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('imu_port'),
            'run_id': LaunchConfiguration('run_id'),
        }],
    )

    cmd_vel_bridge_node = Node(
        package='ballrobot_pkg',
        executable='cmd_vel_bridge.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'run_id': LaunchConfiguration('run_id')}],
    )

    imu_quaternion_node = Node(
        package='ballrobot_pkg',
        executable='imu_quaternion_node.py',
        name='imu_quaternion_node',
        output='screen',
        parameters=[{'run_id': LaunchConfiguration('run_id')}],
        condition=IfCondition(LaunchConfiguration('enable_imu_quat')),
    )

    run_id_log = LogInfo(
        msg=['Run ID for this bringup: ', LaunchConfiguration('run_id'),
             ' (pass --ros-args -p run_id:=', LaunchConfiguration('run_id'),
             ' to any node started separately, e.g. imu_quaternion_node.py '
             'with enable_imu_quat left false, to keep its CSV in sync with '
             'this run)'],
    )

    return LaunchDescription([
        teensy_port_arg,
        uwb_port_arg,
        imu_port_arg,
        enable_imu_quat_arg,
        run_id_arg,
        run_id_log,
        teensy_interface_node,
        uwb_node,
        imu_node,
        cmd_vel_bridge_node,
        imu_quaternion_node,
    ])
