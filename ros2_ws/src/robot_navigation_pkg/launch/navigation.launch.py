"""
Navigation on the robot: sensors, localization, go_to_goal.

    ros2 launch robot_navigation_pkg navigation.launch.py config:=outdoor

Includes ballrobot_pkg bringup.launch.py (sensor nodes and the drive
interface, firmware v4.7) and ballrobot_localization_pkg localization.launch.py
(adapters, static TFs including map -> odom, the EKF), both with the same
shared setup file, then starts go_to_goal with config/nav.yaml.

Arguments
    config    default   shared setup file (ballrobot_pkg/config/<name>.yaml)
    record    false     passed to bringup: record the sensor topics
    sensors   true      false skips bringup (sensors already running)
    nav_config          path to an alternative nav.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sensors = get_package_share_directory('ballrobot_pkg')
    loc = get_package_share_directory('ballrobot_localization_pkg')
    nav = get_package_share_directory('robot_navigation_pkg')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='default'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('nav_config', default_value=os.path.join(nav, 'config', 'nav.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sensors, 'launch', 'bringup.launch.py')),
            launch_arguments={'config': config, 'record': LaunchConfiguration('record')}.items(),
            condition=IfCondition(LaunchConfiguration('sensors')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(loc, 'launch', 'localization.launch.py')),
            launch_arguments={'config': config}.items(),
        ),
        Node(package='robot_navigation_pkg', executable='go_to_goal', name='go_to_goal',
             output='screen', parameters=[LaunchConfiguration('nav_config')]),
    ])
