"""Autonomous navigation for the MicroROS-Pi5 car.

Three ways to run it.

1. Against a saved map, localising with AMCL (the usual case):

       ros2 launch microros_pi5_nav nav2.launch.py map:=my_map

   Then in RViz: "2D Pose Estimate" to tell AMCL roughly where the robot is,
   and "2D Goal Pose" to send it somewhere.

2. On top of live SLAM - map and navigate at the same time, no saved map and no
   AMCL, because slam_toolbox owns map->odom itself:

       ros2 launch microros_pi5_nav slam.launch.py           # terminal 1
       ros2 launch microros_pi5_nav nav2.launch.py slam:=true bringup:=false

3. Mapless - drive to a clicked point using only live lidar, building nothing:

       ros2 launch microros_pi5_nav nav2.launch.py mapless:=true

   Everything runs in the `odom` frame. No map_server, no AMCL, no slam_toolbox,
   so nothing publishes map->odom and nothing is saved. The global costmap
   becomes a rolling window that remembers only what the lidar has swept
   recently. See _write_mapless_params() for the exact differences, and know the
   three limits this buys:

     * Goals must be inside the rolling window (MAPLESS_COSTMAP_SIZE / 2 metres
       of the robot, so ~5 m). Further away, the planner rejects the goal.
     * Obstacles are forgotten once they leave the window, so the planner has no
       memory. A U-shaped dead end will trap the robot - it cannot plan around
       what it can no longer see.
     * Odometry drift is never corrected. Fine for short hops, not for long runs.

   For anything beyond a few metres, build a map (option 2) instead.

`map:=` takes either a bare name (resolved inside this package's maps/ directory)
or an absolute path to a .yaml.

Why this file exists instead of including nav2_bringup's navigation_launch.py
-----------------------------------------------------------------------------
Two deliberate differences:

* **cmd_vel is remapped to cmd_vel_raw.** `cmd_vel_safety` is the only node on
  this robot allowed to publish /cmd_vel. Stock Nav2 would publish it directly
  and you would have two publishers on a motion topic.

* **Nav2's velocity_smoother is not run.** `cmd_vel_safety` already slew-limits
  and dead-mans. Two smoothers in series makes the robot sluggish and makes it
  impossible to tell which one caused a given behaviour.

Environment, as always for this robot:

    export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset CYCLONEDDS_URI
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

# Nav2 servers that make up navigation proper. Order matters: the lifecycle
# manager brings them up in this sequence.
NAV_NODES = [
    ('nav2_controller', 'controller_server', 'controller_server'),
    ('nav2_smoother', 'smoother_server', 'smoother_server'),
    ('nav2_planner', 'planner_server', 'planner_server'),
    ('nav2_behaviors', 'behavior_server', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
    ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
]


# Side of the square rolling global costmap in mapless mode, in metres. The lidar
# reaches 8 m; 10 m of window keeps a margin of already-swept space behind the
# robot without making the grid expensive (10 m / 0.05 m = 200x200 cells).
# This is also the hard limit on goal distance - see the module docstring.
MAPLESS_COSTMAP_SIZE = 10


def _write_mapless_params(params_file):
    """Rewrite the map-based params into an odom-frame, map-free configuration.

    Returns the path to a generated temp file. The shipped YAML is left alone so
    it stays the readable, commented reference for the map-based modes.
    """
    with open(params_file) as f:
        params = yaml.safe_load(f)

    # Every server that plans or recovers relative to the map now works in odom.
    # local_costmap is already on odom, and AMCL's global_frame_id is left alone
    # because AMCL is not launched in this mode.
    for node in ('bt_navigator', 'behavior_server'):
        params[node]['ros__parameters']['global_frame'] = 'odom'

    costmap = params['global_costmap']['global_costmap']['ros__parameters']
    costmap['global_frame'] = 'odom'
    # With no map to cover, the global costmap can only hold what the lidar has
    # seen recently - so it becomes a rolling window like the local one.
    costmap['rolling_window'] = True
    costmap['width'] = MAPLESS_COSTMAP_SIZE
    costmap['height'] = MAPLESS_COSTMAP_SIZE
    # No map means no static layer to read one from.
    costmap['plugins'] = [p for p in costmap['plugins'] if p != 'static_layer']
    costmap.pop('static_layer', None)
    # Unswept cells count as free. Left as unknown, the planner treats the whole
    # window as impassable until the lidar has swept it, and the first goal in
    # any unexplored direction fails.
    costmap['track_unknown_space'] = False

    fd, path = tempfile.mkstemp(prefix='nav2_mapless_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.dump(params, f, default_flow_style=False)
    return path


def _resolve_map(nav_share, map_arg):
    """Accept either a bare map name or a full path to a .yaml."""
    if not map_arg:
        return ''
    if os.path.isabs(map_arg) or map_arg.endswith('.yaml'):
        return map_arg
    return os.path.join(nav_share, 'maps', f'{map_arg}.yaml')


def _setup(context, *args, **kwargs):
    nav_share = get_package_share_directory('microros_pi5_nav')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    slam = LaunchConfiguration('slam').perform(context).lower() in ('true', '1', 'yes')
    mapless = LaunchConfiguration('mapless').perform(context).lower() in ('true', '1', 'yes')
    map_yaml = _resolve_map(nav_share, LaunchConfiguration('map').perform(context))

    if slam and mapless:
        raise RuntimeError(
            'slam:=true and mapless:=true are mutually exclusive. slam builds a map '
            'and owns map->odom; mapless has no map frame at all. Pick one.')

    temporary_params = None
    if mapless:
        # Must happen before RewrittenYaml wraps the file below.
        params_file = _write_mapless_params(params_file)
        temporary_params = params_file
    elif not slam:
        if not map_yaml:
            raise RuntimeError(
                'nav2.launch.py needs map:=<name|path>, slam:=true, or mapless:=true. '
                'With none of them there is nothing to localise against.')
        if not os.path.isfile(map_yaml):
            raise RuntimeError(
                f'map file not found: {map_yaml}\n'
                f'Build a map first with slam.launch.py, save it with '
                f'map_saver_cli into {os.path.join(nav_share, "maps")}, '
                f'then rebuild the package so it installs into share/.')

    # Inject the resolved map path and clock source into the params without
    # editing the YAML on disk.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        },
        convert_types=True,
    )

    common = {
        'output': 'screen',
        'parameters': [configured_params],
    }

    actions = []
    if temporary_params:
        def cleanup(context, *args, **kwargs):
            if os.path.exists(temporary_params):
                os.unlink(temporary_params)
            return []
        actions.append(RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=cleanup)])))

    # --- optional bringup -----------------------------------------------------
    # params_file is passed explicitly. Without it the launch configuration of
    # the same name leaks in from this file - included launch files inherit the
    # parent's configurations - and the bringup nodes come up parameterised by
    # nav2_params.yaml, which defines none of them. They then fall back to code
    # defaults, silently discarding the wheel geometry and cmd_vel_safety's
    # clamps and dead-man timings.
    bringup_share = get_package_share_directory('microros_pi5_bringup')
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(bringup_share, 'config', 'base_params.yaml'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('bringup')),
    ))

    # --- localisation: map_server + AMCL, only in the saved-map mode ---------
    # With slam:=true, slam_toolbox owns map->odom. Running AMCL too would put
    # two publishers on that transform. With mapless:=true there is no map frame
    # to localise into at all - odom is the global frame.
    localization_nodes = []
    if not slam and not mapless:
        actions.append(Node(package='nav2_map_server', executable='map_server',
                            name='map_server', **common))
        actions.append(Node(package='nav2_amcl', executable='amcl',
                            name='amcl', **common))
        localization_nodes = ['map_server', 'amcl']

        actions.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{'use_sim_time': use_sim_time == 'true',
                         'autostart': autostart == 'true',
                         'node_names': localization_nodes}],
        ))

    # --- navigation servers ---------------------------------------------------
    for package, executable, name in NAV_NODES:
        remappings = []
        if name in ('controller_server', 'behavior_server'):
            # Route tracking AND recovery motions through the safety node.
            remappings = [('cmd_vel', 'cmd_vel_raw')]
        actions.append(Node(
            package=package, executable=executable, name=name,
            remappings=remappings, **common))

    actions.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': use_sim_time == 'true',
                     'autostart': autostart == 'true',
                     'node_names': [n for _, _, n in NAV_NODES]}],
    ))

    # --- RViz -----------------------------------------------------------------
    # Mapless gets its own view: Fixed Frame odom, no Map display, no AMCL
    # particles. Pointing nav2.rviz at a mapless run shows an empty screen,
    # because its Fixed Frame `map` never exists.
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if not rviz_config:
        rviz_config = os.path.join(
            nav_share, 'rviz', 'mapless.rviz' if mapless else 'nav2.rviz')

    actions.append(Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time == 'true'}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    ))

    return actions


def generate_launch_description():
    nav_share = get_package_share_directory('microros_pi5_nav')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='',
            description='Map name (resolved in this package\'s maps/) or absolute '
                        'path to a .yaml. Not needed with slam:=true.'),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='Navigate on top of live slam_toolbox instead of a saved '
                        'map. Skips map_server and AMCL.'),
        DeclareLaunchArgument(
            'mapless', default_value='false',
            description='Navigate in the odom frame on live lidar only - no map, no '
                        'SLAM, nothing saved. Goals must be within '
                        f'{MAPLESS_COSTMAP_SIZE // 2} m and the planner has no memory '
                        'of obstacles it can no longer see.'),
        DeclareLaunchArgument(
            'bringup', default_value='true',
            description='Also start bringup.launch.py. Set false when it, or '
                        'slam.launch.py, is already running.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz with the navigation view.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='True only when replaying a bag with /clock.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Let the lifecycle manager activate the stack automatically.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml'),
            description='Nav2 parameters.'),
        DeclareLaunchArgument(
            'rviz_config', default_value='',
            description='RViz configuration. Empty picks mapless.rviz or nav2.rviz '
                        'to match the mode.'),
        OpaqueFunction(function=_setup),
    ])
