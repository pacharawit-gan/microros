"""Keyboard-drive the real robot.

The board is on **ROS_DOMAIN_ID 20** with FastDDS, so every terminal needs:

    export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset CYCLONEDDS_URI

Terminal 1 - the safety layer (this file):

    ros2 launch microros_pi5_bringup teleop.launch.py

Terminal 2 - the keyboard itself. It must be `ros2 run`, not `ros2 launch`: it reads raw
keys from stdin and launch does not give child processes a usable terminal.

    ros2 run microros_pi5_bringup keyboard_teleop

Or use `scripts/drive.sh`, which sets the environment and starts both.

    keyboard_teleop --> /cmd_vel_raw --> [cmd_vel_safety] --> /cmd_vel --> ESP32

Nothing here publishes /cmd_vel directly. cmd_vel_safety stays the only publisher on it -
stop Yahboom's joy stack first or the robot obeys whichever message lands last.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('microros_pi5_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_share, 'config', 'base_params.yaml'),
            description='Parameters for cmd_vel_safety.'),
        DeclareLaunchArgument(
            'start_odom', default_value='false',
            description='Also republish /odom_raw as /odom with a valid stamp and TF. '
                        'Not needed just to drive; turn on if you want RViz to make sense.'),

        Node(
            package='microros_pi5_bringup',
            executable='cmd_vel_safety',
            name='cmd_vel_safety',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='microros_pi5_bringup',
            executable='odom_republisher',
            name='odom_republisher',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('start_odom')),
        ),
    ])
