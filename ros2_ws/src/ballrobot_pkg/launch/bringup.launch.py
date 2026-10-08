from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    # The solenoid node finds the XIAO over BLE by name ("TigerBall-Latch")
    # by default. To pin it to one board, pass its address (shown by
    # `bluetoothctl scan on` or nRF Connect), e.g.:
    #   ros2 launch ballrobot_pkg bringup.launch.py xiao_address:=AA:BB:CC:DD:EE:FF
    xiao_address_arg = DeclareLaunchArgument(
        'xiao_address', default_value='',
        description='BLE address of the feeder-box solenoid XIAO (empty = find by name)')

    # Path is relative to where `ros2 launch` is run (ros2_ws/), so the
    # default plays the repo's audio/ file. Override for another sound:
    #   ros2 launch ballrobot_pkg bringup.launch.py sound_file:=/home/<user>/chicken.mp3
    sound_file_arg = DeclareLaunchArgument(
        'sound_file', default_value='../audio/pig_sound_effect.mp3',
        description='Sound played on /play_sound out of the 3.5 mm jack')

    # imu_quaternion_node is a debug/visualization aid (converts imu/rpy's
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

    # A purely-numeric run_id (default is a timestamp) gets silently
    # coerced to an integer when passed through a launch parameters
    # dict: the params file that's actually handed to each node is YAML,
    # and YAML 1.1's int grammar treats underscores as digit separators
    # (like Python's 1_000_000), so e.g. "20260924_190925" parses as the
    # int 20260924190925 -- which then fails declare_parameter("run_id",
    # "", ...) since the declared default type is str. Wrapping in
    # ParameterValue(..., value_type=str) forces it to stay a string
    # regardless of what it looks like.
    run_id_param = ParameterValue(LaunchConfiguration('run_id'), value_type=str)

    # Opt-in rosbag recording of every sensor/command topic, into
    # bags/run_<run_id> (relative to where `ros2 launch` is run from,
    # i.e. ros2_ws/ -- same as the CSVs). Run via launch rather than a
    # separate terminal so Ctrl-C reaches the recorder and the bag is
    # closed cleanly alongside the nodes:
    #   ros2 launch ballrobot_pkg bringup.launch.py record:=true
    # /imu/data_quat is listed even when enable_imu_quat is false; the
    # recorder just never sees it.
    record_arg = DeclareLaunchArgument(
        'record', default_value='false',
        description='Also record all sensor/command topics with ros2 bag')

    bag_recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '-o', ['bags/run_', LaunchConfiguration('run_id')],
             '/imu/rpy', '/imu/accel', '/imu/data_quat', '/uwb/position',
             '/wheel_speed_meas', '/wheel_speed_cmd', '/cmd_vel'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record')),
    )

    teensy_interface_node = Node(
        package='ballrobot_pkg',
        executable='teensy_interface_node.py',
        name='teensy_interface_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('teensy_port'),
            'run_id': run_id_param,
        }],
    )

    uwb_node = Node(
        package='ballrobot_pkg',
        executable='uwb_node.py',
        name='uwb_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('uwb_port'),
            'run_id': run_id_param,
        }],
    )

    imu_node = Node(
        package='ballrobot_pkg',
        executable='imu_node.py',
        name='imu_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('imu_port'),
            'run_id': run_id_param,
        }],
    )

    cmd_vel_bridge_node = Node(
        package='ballrobot_pkg',
        executable='cmd_vel_bridge.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'run_id': run_id_param}],
    )

    solenoid_release_node = Node(
        package='ballrobot_pkg',
        executable='solenoid_release_node.py',
        name='solenoid_release_node',
        output='screen',
        parameters=[{
            'xiao_address': LaunchConfiguration('xiao_address'),
            'run_id': run_id_param,
        }],
    )

    sound_player_node = Node(
        package='ballrobot_pkg',
        executable='sound_player_node.py',
        name='sound_player_node',
        output='screen',
        parameters=[{'sound_file': LaunchConfiguration('sound_file')}],
    )

    # Streams imu/rpy heading to the operator GUI over BLE (the Pi
    # advertises as "TigerBall-Pi"); see heading_ble_node.py.
    heading_ble_node = Node(
        package='ballrobot_pkg',
        executable='heading_ble_node.py',
        name='heading_ble_node',
        output='screen',
    )

    imu_quaternion_node = Node(
        package='ballrobot_pkg',
        executable='imu_quaternion_node.py',
        name='imu_quaternion_node',
        output='screen',
        parameters=[{'run_id': run_id_param}],
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
        xiao_address_arg,
        sound_file_arg,
        enable_imu_quat_arg,
        run_id_arg,
        record_arg,
        run_id_log,
        teensy_interface_node,
        uwb_node,
        imu_node,
        cmd_vel_bridge_node,
        solenoid_release_node,
        sound_player_node,
        heading_ble_node,
        imu_quaternion_node,
        bag_recorder,
    ])
