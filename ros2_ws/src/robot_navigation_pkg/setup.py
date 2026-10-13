from glob import glob

from setuptools import setup

package_name = 'robot_navigation_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hjbarry',
    maintainer_email='bn5tfo@pm.me',
    description='Point to point navigation and drive calibration for the ball robot.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'calibration_drive = robot_navigation_pkg.calibration_drive:main',
            'align_heading = robot_navigation_pkg.align_heading:main',
            'go_to_goal = robot_navigation_pkg.go_to_goal:main',
        ],
    },
)
