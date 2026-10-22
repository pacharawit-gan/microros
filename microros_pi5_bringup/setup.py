import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'microros_pi5_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pacharawit',
    maintainer_email='akeeslatun@gmail.com',
    description='Bringup nodes and launch files for the Yahboom MicroROS-Pi5 4WD car.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_republisher = microros_pi5_bringup.imu_republisher:main',
            'odom_republisher = microros_pi5_bringup.odom_republisher:main',
            'wheel_joint_publisher = microros_pi5_bringup.wheel_joint_publisher:main',
            'cmd_vel_safety = microros_pi5_bringup.cmd_vel_safety:main',
            'keyboard_teleop = microros_pi5_bringup.keyboard_teleop:main',
        ],
    },
)
