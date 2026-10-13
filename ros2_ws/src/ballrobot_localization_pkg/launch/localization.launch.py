"""
Localization stack: three adapters, two static TFs, ekf_odom and ekf_map.

Does not start any ballrobot_pkg sensor node. Run those with
ballrobot_pkg/bringup.launch.py (live) or replay them from a bag
(offline.launch.py).

Launch arguments
    use_sim_time      false   true when replaying a bag with --clock
    two_d_mode        false   robot_localization two_d_mode for both EKFs
    fuse_imu_yaw      true    fuse absolute IMU yaw in ekf_map
    fuse_imu_accel    false   fuse IMU linear acceleration in both EKFs
    use_wheel_odom    true    false clears odom0_config in both EKFs (run C)
    uwb_dropout       ''      'start,end[,start,end...]' seconds, overrides
                              the uwb_pose_adapter dropout_windows (run D)
    config            default shared setup file, same argument as
                              ballrobot_pkg/bringup.launch.py: a name in
                              ballrobot_pkg/config/ or a path. Supplies the
                              anchor layout and optional adapter overrides
                              (see ballrobot_localization_pkg/setup_config.py)
    adapters_config, ekf_config   parameter file paths
    imu_x/y/z/roll/pitch/yaw, uwb_x/y/z   base_link offsets (m, rad)
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
IMU_YAW = 5
IMU_ACCEL = (12, 13, 14)

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


def _ekf_params(context, ekf_file, node_name):
    with open(ekf_file) as f:
        params = dict(yaml.safe_load(f)[node_name]['ros__parameters'])

    params['use_sim_time'] = _bool(context, 'use_sim_time')
    params['two_d_mode'] = _bool(context, 'two_d_mode')

    imu_cfg = list(params['imu0_config'])
    if node_name == 'ekf_map':
        imu_cfg[IMU_YAW] = _bool(context, 'fuse_imu_yaw')
    if not _bool(context, 'fuse_imu_accel'):
        for i in IMU_ACCEL:
            imu_cfg[i] = False
    params['imu0_config'] = imu_cfg

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
    anchors = setup_config.anchor_positions(setup_config.load(setup_file))
    anchor_param = {'anchor_positions': anchors} if anchors else {}
    log = LogInfo(msg=f'Setup config: {setup_file}, UWB anchors: {anchors or "from adapters.yaml"}')

    uwb_overrides = dict(sim)
    dropout = LaunchConfiguration('uwb_dropout').perform(context).strip()
    if dropout:
        uwb_overrides['dropout_windows'] = [float(v) for v in dropout.split(',')]

    adapters = [
        Node(package=PKG, executable='uwb_pose_adapter', name='uwb_pose_adapter',
             output='screen',
             parameters=[adapters_file, anchor_param, setup_file, uwb_overrides]),
        Node(package=PKG, executable='imu_adapter', name='imu_adapter',
             output='screen', parameters=[adapters_file, setup_file, sim]),
        Node(package=PKG, executable='wheel_odom_adapter', name='wheel_odom_adapter',
             output='screen', parameters=[adapters_file, setup_file, sim]),
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
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_imu_tf', arguments=tf_args('imu', 'imu_link'), parameters=[sim]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_uwb_tag_tf', arguments=tf_args('uwb', 'uwb_tag_link', False),
             parameters=[sim]),
    ]

    ekfs = [
        Node(package='robot_localization', executable='ekf_node', name='ekf_odom',
             output='screen', parameters=[_ekf_params(context, ekf_file, 'ekf_odom')],
             remappings=[('odometry/filtered', '/odometry/local')]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_map',
             output='screen', parameters=[_ekf_params(context, ekf_file, 'ekf_map')],
             remappings=[('odometry/filtered', '/odometry/global')]),
    ]
    return [log] + adapters + static_tfs + ekfs


def generate_launch_description():
    share = get_package_share_directory(PKG)
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('two_d_mode', default_value='false'),
        DeclareLaunchArgument('fuse_imu_yaw', default_value='true'),
        DeclareLaunchArgument('fuse_imu_accel', default_value='false'),
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
