# mypy: disable-error-code="import-untyped"

from setuptools import find_packages, setup

package_name = 'kibot_one_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/follow_phase1.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jese--ki',
    maintainer_email='2094901072@qq.com',
    description='机器人核心逻辑',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_watchdog = kibot_one_control.cmd_vel_watchdog:main',
            'keyboard_teleop = kibot_one_control.keyboard_teleop:main',
            'mode_control = kibot_one_control.mode_control:main',
            'flag_pose_publisher = kibot_one_control.flag_pose_publisher:main',
            'follow_controller = kibot_one_control.follow_controller:main',
        ],
    },
)
