"""
Localization stack: three adapters, static TFs and one EKF (config/ekf.yaml).

TF tree: map -(static identity)-> odom -(ekf)-> base_link -(static)->
{imu_link, uwb_tag_link}. The EKF publishes /odometry/filtered.

Does not start any ballrobot_pkg sensor node. Run those with
ballrobot_pkg/bringup.launch.py (live) or replay them from a bag
(offline.launch.py).

Launch arguments
    use_sim_time      false   true when replaying a bag with --clock
    two_d_mode        true    robot_localization two_d_mode
    use_imu           true    false clears imu0_config (comparison run B)
    use_wheel_odom    true    false clears odom0_config (comparison run C)
    uwb_dropout       ''      'start,end[,start,end...]' seconds, overrides
                              the uwb_pose_adapter dropout_windows (run D)
    config            default shared setup file, same argument as
                              ballrobot_pkg/bringup.launch.py: a name in
                              ballrobot_pkg/config/ or a path. Supplies the
                              anchor layout and optional adapter overrides
                              (see ballrobot_localization_pkg/setup_config.py)
    adapters_config, ekf_config   parameter file paths
    imu_x/y/z/roll/pitch/yaw, uwb_x/y/z   base_link offsets (m, rad). The IMU
                              rotation is also given to imu_adapter, which
                              needs it to express the yaw rate correctly.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ballrobot_localization_pkg import setup_config

PKG = 'ballrobot_localization_pkg'

# Static TF defaults, metres and radians, base_link to sensor.
# TODO: measure all offsets on the robot.
# imu_roll defaults to pi, not zero: every recorded bag shows the BNO085
# reading about 179 deg roll at rest, so the board is mounted upside down.
STATIC_TF_DEFAULTS = {
    'imu_x': '0.0', 'imu_y': '0.0', 'imu_z': '0.0',
    'imu_roll': '3.141592653589793', 'imu_pitch': '0.0', 'imu_yaw': '0.0',
    'uwb_x': '0.0', 'uwb_y': '0.0', 'uwb_z': '0.0',
}


def _bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('true', '1', 'yes')


def _ekf_params(context, ekf_file):
    with open(ekf_file) as f:
        params = dict(yaml.safe_load(f)['ekf']['ros__parameters'])

    params['use_sim_time'] = _bool(context, 'use_sim_time')
    params['two_d_mode'] = _bool(context, 'two_d_mode')
    if not _bool(context, 'use_imu'):
        params['imu0_config'] = [False] * 15
    if not _bool(context, 'use_wheel_odom'):
        params['odom0_config'] = [False] * 15
    return params


def _launch_setup(context, *args, **kwargs):
    adapters_file = LaunchConfiguration('adapters_config').perform(context)
    ekf_file = LaunchConfiguration('ekf_config').perform(context)
    sim = {'use_sim_time': _bool(context, 'use_sim_time')}

    # Shared setup file. Order of precedence, lowest first: adapters.yaml,
    # anchors from the setup's uwb_node block, the setup's own adapter
    # blocks, then launch arguments.
    setup_file = setup_config.resolve(LaunchConfiguration('config').perform(context))
    setup = setup_config.load(setup_file)
    anchors = setup_config.anchor_positions(setup)
    anchor_param = {'anchor_positions': anchors} if anchors else {}
    geometry_param = setup_config.drive_geometry(setup)
    log = LogInfo(msg=f'Setup config: {setup_file}, UWB anchors: {anchors or "from adapters.yaml"}, '
                      f'wheel geometry: {geometry_param or "from adapters.yaml"}')

    mount = {f'mount_{k}': float(LaunchConfiguration(f'imu_{k}').perform(context))
             for k in ('roll', 'pitch', 'yaw')}

    uwb_overrides = dict(sim)
    dropout = LaunchConfiguration('uwb_dropout').perform(context).strip()
    if dropout:
        uwb_overrides['dropout_windows'] = [float(v) for v in dropout.split(',')]

    adapters = [
        Node(package=PKG, executable='uwb_pose_adapter', name='uwb_pose_adapter',
             output='screen',
             parameters=[adapters_file, anchor_param, setup_file, uwb_overrides]),
        Node(package=PKG, executable='imu_adapter', name='imu_adapter',
             output='screen', parameters=[adapters_file, mount, setup_file, sim]),
        Node(package=PKG, executable='wheel_odom_adapter', name='wheel_odom_adapter',
             output='screen', parameters=[adapters_file, geometry_param, setup_file, sim]),
    ]

    def tf_args(prefix, child, rotation=True):
        args = ['--x', LaunchConfiguration(f'{prefix}_x'),
                '--y', LaunchConfiguration(f'{prefix}_y'),
                '--z', LaunchConfiguration(f'{prefix}_z')]
        if rotation:
            args += ['--roll', LaunchConfiguration(f'{prefix}_roll'),
                     '--pitch', LaunchConfiguration(f'{prefix}_pitch'),
                     '--yaw', LaunchConfiguration(f'{prefix}_yaw')]
        return args + ['--frame-id', 'base_link', '--child-frame-id', child]

    static_tfs = [
        # odom equals the UWB map frame (single EKF, world_frame odom).
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='map_to_odom_tf', arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
             parameters=[sim]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_imu_tf', arguments=tf_args('imu', 'imu_link'), parameters=[sim]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_uwb_tag_tf', arguments=tf_args('uwb', 'uwb_tag_link', False),
             parameters=[sim]),
    ]

    # Subscribes set_pose (resolves to /set_pose), used by the alignment drive.
    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf',
               output='screen', parameters=[_ekf_params(context, ekf_file)],
               remappings=[('odometry/filtered', '/odometry/filtered')])
    return [log] + adapters + static_tfs + [ekf]


def generate_launch_description():
    share = get_package_share_directory(PKG)
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('two_d_mode', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_wheel_odom', default_value='true'),
        DeclareLaunchArgument('uwb_dropout', default_value=''),
        DeclareLaunchArgument(
            'config', default_value='default',
            description='Shared setup file: name in ballrobot_pkg/config/ or a path'),
        DeclareLaunchArgument(
            'adapters_config', default_value=os.path.join(share, 'config', 'adapters.yaml')),
        DeclareLaunchArgument(
            'ekf_config', default_value=os.path.join(share, 'config', 'ekf.yaml')),
    ]
    args += [DeclareLaunchArgument(k, default_value=v) for k, v in STATIC_TF_DEFAULTS.items()]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
