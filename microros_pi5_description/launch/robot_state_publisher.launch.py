"""Load the xacro and start robot_state_publisher.

Kept separate from display.launch.py so that bringup can reuse exactly the same
description without dragging in RViz or the joint-state GUI.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
            FindPackageShare('microros_pi5_description'), 'urdf', 'microros_pi5.urdf.xacro',
        ]),
        description='Absolute path to the robot xacro/urdf file.',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock instead of wall time.',
    )

    # ParameterValue(..., value_type=str) is required: without it the launch system
    # tries to guess a type for the xacro output and mangles the XML.
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([model_arg, use_sim_time_arg, robot_state_publisher])
