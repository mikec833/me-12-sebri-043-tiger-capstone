"""
Offline replay: localization.launch.py with use_sim_time true.

Run alongside
    ros2 bag play <bag> --clock
The bag supplies the raw sensor topics; only ballrobot_localization_pkg nodes run.
Every localization.launch.py argument can still be passed here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('ballrobot_localization_pkg')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'localization.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
    ])
