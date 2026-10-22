"""Offline model viewer - the equivalent of Yahboom's `yahboomcar_description display_launch.py`.

    ros2 launch microros_pi5_description display.launch.py

Starts robot_state_publisher + joint_state_publisher_gui + RViz2. Touches no hardware:
the sliders publish /joint_states, robot_state_publisher turns those into /tf.

Requires joint_state_publisher_gui:
    sudo apt install ros-humble-joint-state-publisher-gui
Run with gui:=false if you have not installed it (wheels will then sit at zero).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('microros_pi5_description')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Start joint_state_publisher_gui (sliders) instead of the headless publisher.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Start RViz2.',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz']),
        description='RViz2 config file.',
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'robot_state_publisher.launch.py'])
        ),
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        rviz_config_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz,
    ])
