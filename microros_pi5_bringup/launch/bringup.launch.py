"""Bring the MicroROS-Pi5 car up as a proper ROS 2 robot.

    # minimum: agent already running elsewhere
    ros2 launch microros_pi5_bringup bringup.launch.py

    # start the micro-ROS agent too
    ros2 launch microros_pi5_bringup bringup.launch.py start_agent:=true

    # fuse the IMU (needs ros-humble-robot-localization)
    ros2 launch microros_pi5_bringup bringup.launch.py use_ekf:=true

Resulting graph:

    /cmd_vel_raw --> [cmd_vel_safety] --> /cmd_vel ----------------> ESP32 board
    ESP32 board --> /odom_raw --> [odom_republisher] --> /odom  (+ odom->base_footprint TF)
                            \\--> [wheel_joint_publisher] --> /joint_states
                                                                  |
                              /robot_description --> [robot_state_publisher] --> /tf

With use_ekf:=true the TF ownership moves: odom_republisher stops broadcasting and
ekf_filter_node publishes odom -> base_footprint instead. Exactly one owner, always.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory('microros_pi5_bringup')
    description_share = get_package_share_directory('microros_pi5_description')

    params_file = LaunchConfiguration('params_file').perform(context)
    ekf_params_file = LaunchConfiguration('ekf_params_file').perform(context)
    use_ekf = LaunchConfiguration('use_ekf').perform(context).lower() in ('true', '1', 'yes')
    namespace = LaunchConfiguration('namespace').perform(context)

    if namespace:
        raise RuntimeError('This stack supports one robot per ROS_DOMAIN_ID only; '
                           'namespace is not supported because TF frames are shared.')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    clock_params = {'use_sim_time': use_sim_time}
    actions = []

    # --- micro-ROS agent (optional) ------------------------------------------------
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'micro_ros_agent.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_agent')),
    ))

    # --- URDF + robot_state_publisher ----------------------------------------------
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': str(use_sim_time).lower()}.items(),
        condition=IfCondition(LaunchConfiguration('start_description')),
    ))

    # --- odometry ------------------------------------------------------------------
    # publish_tf is decided here, not in the yaml, so that use_ekf stays a single switch.
    actions.append(Node(
        package='microros_pi5_bringup',
        executable='odom_republisher',
        name='odom_republisher',
        output='screen',
        parameters=[params_file, clock_params, {'publish_tf': not use_ekf,
                    'output_topic': 'odom_wheel' if use_ekf else 'odom'}],
    ))

    # --- joint states for the wheels + gimbal ---------------------------------------
    actions.append(Node(
        package='microros_pi5_bringup',
        executable='wheel_joint_publisher',
        name='wheel_joint_publisher',
        output='screen',
        parameters=[params_file, clock_params],
    ))

    # --- cmd_vel rate limiting + dead-man -------------------------------------------
    actions.append(Node(
        package='microros_pi5_bringup',
        executable='cmd_vel_safety',
        name='cmd_vel_safety',
        output='screen',
        parameters=[params_file, clock_params],
    ))

    # --- EKF (optional) --------------------------------------------------------------
    if use_ekf:
        actions.append(Node(
            package='microros_pi5_bringup', executable='imu_republisher',
            name='imu_republisher', output='screen', parameters=[clock_params],
        ))
        actions.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file, clock_params],
            remappings=[('odometry/filtered', 'odom')],
        ))

    return [GroupAction(actions)]


def generate_launch_description():
    bringup_share = get_package_share_directory('microros_pi5_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Must be empty. Use separate ROS_DOMAIN_ID values for separate robots.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock for sensor processing and TF.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_share, 'config', 'base_params.yaml'),
            description='Parameters for the bringup nodes.'),
        DeclareLaunchArgument(
            'ekf_params_file',
            default_value=os.path.join(bringup_share, 'config', 'ekf.yaml'),
            description='robot_localization parameters (use_ekf:=true only).'),
        DeclareLaunchArgument(
            'start_agent', default_value='false',
            description='Also start the micro-ROS agent. Leave false if you run Yahboom\'s '
                        'start_agent script or the agent container separately.'),
        DeclareLaunchArgument(
            'start_description', default_value='true',
            description='Start robot_state_publisher with the MicroROS-Pi5 URDF.'),
        DeclareLaunchArgument(
            'use_ekf', default_value='false',
            description='Fuse sanitized wheel odometry and gyro via robot_localization. Requires '
                        'ros-humble-robot-localization.'),
        OpaqueFunction(function=_setup),
    ])
