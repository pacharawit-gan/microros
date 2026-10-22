"""Start the micro-ROS agent - the bridge between the ESP32 board and real DDS.

Nothing from the control board appears on the ROS 2 graph until this is running.

    ros2 launch microros_pi5_bringup micro_ros_agent.launch.py

Needs the agent installed natively:

    sudo apt install ros-humble-micro-ros-agent      # if available for your setup
    # otherwise build micro_ros_setup, or use Yahboom's container:
    sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \\
         microros/micro-ros-agent:humble udp4 --port 8090 -v4

Transport defaults to the observed USB serial setup at /dev/ttyUSB0, 921600 baud.
Use transport:=udp4 port:=8090 only for firmware configured for UDP.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _agent_node(context, *args, **kwargs):
    transport = LaunchConfiguration('transport').perform(context)
    verbosity = LaunchConfiguration('verbosity').perform(context)

    if transport == 'serial':
        agent_args = ['serial', '--dev',
                      LaunchConfiguration('device').perform(context),
                      '-b', LaunchConfiguration('baudrate').perform(context)]
    else:
        agent_args = ['udp4', '--port', LaunchConfiguration('port').perform(context)]

    agent_args.append(f'-v{verbosity}')

    return [Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=agent_args,
        emulate_tty=True,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('transport', default_value='serial',
                              choices=['udp4', 'serial'],
                              description='WiFi (udp4) or USB (serial).'),
        DeclareLaunchArgument('port', default_value='8090',
                              description='UDP port. Must match the board configuration.'),
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0',
                              description='Serial device (transport:=serial only).'),
        DeclareLaunchArgument('baudrate', default_value='921600',
                              description='Serial baud rate (transport:=serial only).'),
        DeclareLaunchArgument('verbosity', default_value='4',
                              description='Agent verbosity 0-6. 4 shows session setup.'),
        OpaqueFunction(function=_agent_node),
    ])
