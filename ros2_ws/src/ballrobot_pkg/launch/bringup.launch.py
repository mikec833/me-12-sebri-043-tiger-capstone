import glob
import os
import shutil
from datetime import datetime

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# USB name patterns under /dev/serial/by-id/, tried in order. Unlike
# /dev/ttyACM<n>, which is handed out in plug-in/boot order and swaps
# between runs, these names are fixed per device.
MOTOR_BOARD_PORT_PATTERNS = ['usb-Seeed_XIAO_nRF52840*']
UWB_PORT_PATTERNS = ['usb-Nordic_Semiconductor_nRF52*']


def find_serial_port(patterns, fallback):
    """First /dev/serial/by-id/ entry matching a pattern, else fallback."""
    for pattern in patterns:
        matches = sorted(glob.glob(f'/dev/serial/by-id/{pattern}'))
        if matches:
            return matches[0]
    return fallback


def resolve_config(context):
    """Resolve the `config` launch argument to a parameter file path.

    A bare name (e.g. `outdoor`) means config/<name>.yaml in this
    package's installed share directory; anything containing a `/` or
    ending in .yaml is taken as a path. The resolved path is stored as
    the `config_file` launch configuration (read by every node below),
    and a copy is saved next to the run's CSVs so each run records the
    setup it used.
    """
    config = context.launch_configurations['config'].strip()
    config_dir = os.path.join(get_package_share_directory('ballrobot_pkg'), 'config')
    if '/' in config or config.endswith('.yaml'):
        path = os.path.abspath(os.path.expanduser(config))
    else:
        path = os.path.join(config_dir, f'{config}.yaml')
    if not os.path.isfile(path):
        available = sorted(
            f[:-len('.yaml')] for f in os.listdir(config_dir) if f.endswith('.yaml'))
        raise RuntimeError(
            f"Config '{config}' not found ({path}). Available: {', '.join(available)}. "
            "Rebuild with colcon build after adding a new file to config/.")
    context.launch_configurations['config_file'] = path

    # Same folder the nodes' CSVs land in (see run_logging.py), relative
    # to where `ros2 launch` is run (ros2_ws/)
    run_id = context.launch_configurations['run_id']
    run_dir = os.path.join('src', 'outputs', f'run_{run_id}')
    os.makedirs(run_dir, exist_ok=True)
    shutil.copy(path, os.path.join(run_dir, f'config_{os.path.basename(path)}'))

    return [LogInfo(msg=f'Config: {path}')]


def generate_launch_description():
    # The motor board (drive XIAO) and the UWB tag are found
    # by their fixed USB names (see *_PORT_PATTERNS above), falling back to
    # ttyACM0/ttyACM1 if not found. Check with `ls -l /dev/serial/by-id/`;
    # override either one explicitly if needed, e.g.:
    #   ros2 launch ballrobot_pkg bringup.launch.py teensy_port:=/dev/ttyACM1 uwb_port:=/dev/ttyACM0
    teensy_port_arg = DeclareLaunchArgument(
        'teensy_port',
        default_value=find_serial_port(MOTOR_BOARD_PORT_PATTERNS, '/dev/ttyACM0'),
        description='Serial port for the drive XIAO (encoder MEAS lines)')
    uwb_port_arg = DeclareLaunchArgument(
        'uwb_port',
        default_value=find_serial_port(UWB_PORT_PATTERNS, '/dev/ttyACM1'),
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

    # Per-node parameters (anchor layout etc.) come from a YAML file in
    # config/, picked by name, e.g. for config/outdoor.yaml:
    #   ros2 launch ballrobot_pkg bringup.launch.py config:=outdoor
    # or by path: config:=/home/<user>/my_setup.yaml. Every node gets the
    # file but only reads its own block; see config/default.yaml.
    config_arg = DeclareLaunchArgument(
        'config', default_value='default',
        description='Parameter file: name in config/ (no .yaml) or a path')
    config_file = LaunchConfiguration('config_file')

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
             '/wheel_states', '/wheel_speed_cmd', '/cmd_vel',
             '/feeder/distance', '/feeder_box_threshold_reached',
             '/solenoid_released', '/viz/pose'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record')),
    )

    teensy_interface_node = Node(
        package='ballrobot_pkg',
        executable='teensy_interface_node.py',
        name='teensy_interface_node',
        output='screen',
        parameters=[config_file, {
            'serial_port': LaunchConfiguration('teensy_port'),
            'run_id': run_id_param,
        }],
    )

    uwb_node = Node(
        package='ballrobot_pkg',
        executable='uwb_node.py',
        name='uwb_node',
        output='screen',
        parameters=[config_file, {
            'serial_port': LaunchConfiguration('uwb_port'),
            'run_id': run_id_param,
        }],
    )

    imu_node = Node(
        package='ballrobot_pkg',
        executable='imu_node.py',
        name='imu_node',
        output='screen',
        parameters=[config_file, {
            'serial_port': LaunchConfiguration('imu_port'),
            'run_id': run_id_param,
        }],
    )

    cmd_vel_bridge_node = Node(
        package='ballrobot_pkg',
        executable='cmd_vel_bridge.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[config_file, {'run_id': run_id_param}],
    )

    # Watches uwb/position and publishes /feeder_box_threshold_reached
    # once the robot is within trigger_radius_m of feeder_position (both
    # set in the config YAML); solenoid_release_node then sends the
    # OPEN command over BLE. See feeder_trigger_node.py.
    feeder_trigger_node = Node(
        package='ballrobot_pkg',
        executable='feeder_trigger_node.py',
        name='feeder_trigger_node',
        output='screen',
        parameters=[config_file],
    )

    solenoid_release_node = Node(
        package='ballrobot_pkg',
        executable='solenoid_release_node.py',
        name='solenoid_release_node',
        output='screen',
        parameters=[config_file, {
            'xiao_address': LaunchConfiguration('xiao_address'),
            'run_id': run_id_param,
        }],
    )

    sound_player_node = Node(
        package='ballrobot_pkg',
        executable='sound_player_node.py',
        name='sound_player_node',
        output='screen',
        parameters=[config_file, {'sound_file': LaunchConfiguration('sound_file')}],
    )

    # foxglove_bridge serves every topic over a WebSocket on port 8765,
    # for the Foxglove app (ws://<pi>:8765) and the operator GUI's
    # heading compass (reads imu/rpy). This replaces heading_ble_node.py,
    # so the Pi's Bluetooth is left free for solenoid_release_node.
    # Install once with: sudo apt install ros-$ROS_DISTRO-foxglove-bridge
    # Turn off with foxglove:=false.
    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Start foxglove_bridge (WebSocket, port 8765) for Foxglove and the GUI')
    try:
        get_package_share_directory('foxglove_bridge')
        foxglove_bridge = Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': 8765}],
            condition=IfCondition(LaunchConfiguration('foxglove')),
        )
    except PackageNotFoundError:
        foxglove_bridge = LogInfo(
            msg='foxglove_bridge not installed -- no Foxglove/GUI heading link. Install with: '
                'sudo apt install ros-$ROS_DISTRO-foxglove-bridge',
            condition=IfCondition(LaunchConfiguration('foxglove')),
        )

    # Combines uwb/position and imu/rpy yaw into one PoseStamped on
    # viz/pose, so Foxglove's 3D panel (via foxglove_bridge above) shows
    # the robot's live position and heading as a single arrow on the XY
    # plane; see pose_viz_node.py. On by default; turn it off with:
    #   ros2 launch ballrobot_pkg bringup.launch.py viz:=false
    viz_arg = DeclareLaunchArgument(
        'viz', default_value='true',
        description='Start pose_viz_node (live pose and heading for Foxglove)')

    pose_viz_node = Node(
        package='ballrobot_pkg',
        executable='pose_viz_node.py',
        name='pose_viz_node',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(LaunchConfiguration('viz')),
    )

    imu_quaternion_node = Node(
        package='ballrobot_pkg',
        executable='imu_quaternion_node.py',
        name='imu_quaternion_node',
        output='screen',
        parameters=[config_file, {'run_id': run_id_param}],
        condition=IfCondition(LaunchConfiguration('enable_imu_quat')),
    )

    port_log = LogInfo(
        msg=['Motor board port: ', LaunchConfiguration('teensy_port'),
             ' | UWB port: ', LaunchConfiguration('uwb_port')],
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
        foxglove_arg,
        viz_arg,
        config_arg,
        # Must come before the nodes: sets config_file for them
        OpaqueFunction(function=resolve_config),
        run_id_log,
        port_log,
        teensy_interface_node,
        uwb_node,
        imu_node,
        cmd_vel_bridge_node,
        feeder_trigger_node,
        solenoid_release_node,
        sound_player_node,
        foxglove_bridge,
        pose_viz_node,
        imu_quaternion_node,
        bag_recorder,
    ])
