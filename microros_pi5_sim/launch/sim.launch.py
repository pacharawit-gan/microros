"""Drive the MicroROS-Pi5 around without the robot.

    ros2 launch microros_pi5_sim sim.launch.py

Then, in a second terminal, drive it:

    ros2 run microros_pi5_bringup keyboard_teleop

(teleop is not launched here on purpose - a node started by a launch file has no terminal
to read keys from.)

What runs:

    [fake_board]  <- /cmd_vel ------------------ [cmd_vel_safety] <- /cmd_vel_raw <- teleop
         │
         ├── /odom_raw ──► [odom_republisher] ──► /odom + TF odom→base_footprint
         │            └───► [wheel_joint_publisher] ──► /joint_states
         └── /imu                                            │
                            /robot_description ──► [robot_state_publisher] ──► /tf

Every node above except fake_board is the same one that runs on the real car. Swap
fake_board for the micro-ROS agent and nothing else changes.

RViz opens with Fixed Frame = odom. That matters: with Fixed Frame = base_footprint the
camera rides along with the robot and it looks like the world is moving instead.
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
    sim_share = get_package_share_directory('microros_pi5_sim')
    bringup_share = get_package_share_directory('microros_pi5_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Start RViz2.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(sim_share, 'rviz', 'sim.rviz'),
            description='RViz2 config.'),
        DeclareLaunchArgument(
            'sim_params_file',
            default_value=os.path.join(sim_share, 'config', 'fake_board.yaml'),
            description='Parameters for the fake control board.'),
        DeclareLaunchArgument(
            'use_ekf', default_value='false',
            description='Fuse the fake IMU via robot_localization (needs the package).'),

        # The real bringup stack, unmodified. start_agent stays false: fake_board is
        # standing in for the board, so there is nothing for an agent to bridge to.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'bringup.launch.py')),
            launch_arguments={
                'start_agent': 'false',
                'start_description': 'true',
                'use_ekf': LaunchConfiguration('use_ekf'),
            }.items(),
        ),

        Node(
            package='microros_pi5_sim',
            executable='fake_board',
            name='fake_board',
            output='screen',
            parameters=[LaunchConfiguration('sim_params_file')],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
