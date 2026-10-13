"""
Offline EKF tuning: the localization stack on sim time, for use with

    ros2 bag play <bag> --clock

The bag supplies the raw sensor topics (/uwb/position, /imu/rpy, /imu/accel,
/wheel_states); only the adapters and the EKF run. Every
ballrobot_localization_pkg localization.launch.py argument can be passed,
e.g. config:=<setup the bag was recorded with>, use_imu:=false.
Evaluate the result with ballrobot_localization_pkg/scripts/evaluate.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    loc = get_package_share_directory('ballrobot_localization_pkg')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(loc, 'launch', 'localization.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
    ])
