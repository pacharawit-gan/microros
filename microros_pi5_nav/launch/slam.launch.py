"""Build a map of the room with slam_toolbox, driving the robot by hand.

    # everything in one shot: bringup + SLAM + RViz
    ros2 launch microros_pi5_nav slam.launch.py

    # SLAM only, because bringup.launch.py is already running elsewhere
    ros2 launch microros_pi5_nav slam.launch.py bringup:=false

Then, in another terminal, drive it:

    ros2 run microros_pi5_bringup keyboard_teleop

`keyboard_teleop` must be `ros2 run`, never `ros2 launch` - it reads raw keys from
stdin and launch does not hand children a usable terminal.

When the map looks complete, save it WITHOUT stopping this launch:

    ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/microros_pi5/microros_pi5_nav/maps/my_map

That writes my_map.pgm + my_map.yaml. Rebuild the package afterwards so the map
installs into share/ and `nav2.launch.py map:=my_map` can find it by bare name.

Remember the environment - this robot is on domain 20 with FastDDS:

    export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset CYCLONEDDS_URI
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
    nav_share = get_package_share_directory('microros_pi5_nav')
    bringup_share = get_package_share_directory('microros_pi5_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bringup', default_value='true',
            description='Also start bringup.launch.py (odometry, TF, cmd_vel safety). '
                        'Set false if you already have it running in another terminal.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz with a map-building view.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='True only when replaying a bag with /clock.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav_share, 'config', 'slam_toolbox.yaml'),
            description='slam_toolbox parameters.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(nav_share, 'rviz', 'slam.rviz'),
            description='RViz configuration.'),

        # params_file is passed explicitly. Without it this file's params_file
        # (slam_toolbox.yaml) leaks in - included launch files inherit the
        # parent's launch configurations - and the bringup nodes come up
        # parameterised by a YAML that defines none of them, silently falling
        # back to code defaults for the wheel geometry and cmd_vel_safety limits.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'bringup.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': os.path.join(
                    bringup_share, 'config', 'base_params.yaml'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('bringup')),
        ),

        # Node name must stay 'slam_toolbox' - it is the top-level key in the YAML,
        # and a renamed node silently falls back to built-in defaults.
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
