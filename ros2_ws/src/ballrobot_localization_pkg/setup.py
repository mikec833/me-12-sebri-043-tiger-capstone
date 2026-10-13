from glob import glob

from setuptools import setup

package_name = 'ballrobot_localization_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'DISCOVERY.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hjbarry',
    maintainer_email='bn5tfo@pm.me',
    description='Dual EKF localization (UWB, IMU, wheel odometry) for the ball robot.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'uwb_pose_adapter = ballrobot_localization_pkg.uwb_pose_adapter:main',
            'imu_adapter = ballrobot_localization_pkg.imu_adapter:main',
            'wheel_odom_adapter = ballrobot_localization_pkg.wheel_odom_adapter:main',
        ],
    },
)
