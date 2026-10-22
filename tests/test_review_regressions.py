"""Offline regression checks; no ROS nodes or hardware connections are started."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[1]


def load(relative):
    spec = importlib.util.spec_from_file_location('review_module', ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_keyboard_stop_and_reverse():
    module = load('microros_pi5_bringup/microros_pi5_bringup/keyboard_teleop.py')
    state = SimpleNamespace(x=1.0, th=1.0, last_key_time=123,
                            get_clock=lambda: SimpleNamespace(now=lambda: 456))
    for key in ('k', ' '):
        state.x = state.th = 1.0
        assert module.KeyboardTeleop.apply_key(state, key)
        assert (state.x, state.th, state.last_key_time) == (0.0, 0.0, None)
    for key in (',', 's', 'DOWN'):
        assert module.KeyboardTeleop.apply_key(state, key)
        assert (state.x, state.th) == (-1.0, 0.0)


@pytest.mark.parametrize('mode', ['mapless', 'slam', 'saved_map'])
def test_navigation_commands_pass_through_safety(mode, tmp_path):
    module = load('microros_pi5_nav/launch/nav2.launch.py')
    context = LaunchContext()
    map_file = tmp_path / 'map.yaml'
    map_file.write_text('image: map.pgm\n')
    context.launch_configurations.update({
        'use_sim_time': 'false', 'autostart': 'true',
        'params_file': str(ROOT / 'microros_pi5_nav/config/nav2_params.yaml'),
        'slam': str(mode == 'slam').lower(),
        'mapless': str(mode == 'mapless').lower(),
        'map': str(map_file) if mode == 'saved_map' else '',
        'rviz_config': '',
    })
    def share(name):
        return str(ROOT / name)
    with patch.object(module, 'get_package_share_directory', side_effect=share), \
            patch.object(module, 'Node', side_effect=lambda **kwargs: kwargs):
        actions = module._setup(context)
    nodes = {a['name']: a for a in actions if isinstance(a, dict)}
    for name in ('controller_server', 'behavior_server'):
        assert ('cmd_vel', 'cmd_vel_raw') in nodes[name]['remappings']
    assert ('amcl' in nodes) == (mode == 'saved_map')


def safety_node(monkeypatch, overrides=None):
    from unittest.mock import Mock
    module = load('microros_pi5_bringup/microros_pi5_bringup/cmd_vel_safety.py')
    params = dict(overrides or {})
    monkeypatch.setattr(module.Node, '__init__', lambda *args: None)
    cls = module.CmdVelSafety
    monkeypatch.setattr(cls, 'declare_parameter', lambda self, key, default: params.setdefault(key, default))
    monkeypatch.setattr(cls, 'get_parameter', lambda self, key: SimpleNamespace(value=params[key]))
    monkeypatch.setattr(cls, 'create_subscription', lambda *args: None)
    monkeypatch.setattr(cls, 'create_publisher', lambda *args: Mock())
    monkeypatch.setattr(cls, 'get_logger', lambda self: Mock())
    timer = Mock()
    monkeypatch.setattr(cls, 'create_timer', timer)
    return cls(), timer


@pytest.mark.parametrize('field', ['linear', 'angular'])
@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_command_stops_whole_command(monkeypatch, field, value):
    from geometry_msgs.msg import Twist
    node, _ = safety_node(monkeypatch)
    msg = Twist()
    msg.linear.x = 0.3
    node.on_cmd(msg)
    setattr(getattr(msg, field), 'z', value)
    node.on_cmd(msg)
    assert node.target == Twist()
    assert node.last_input_time is None
    node.tick()
    assert node.pub.publish.call_args.args[0] == Twist()


@pytest.mark.parametrize('name,value', [('rate', 0.0), ('timeout', -1.0),
                                      ('max_linear_accel', 0.0), ('max_angular_z', float('nan'))])
def test_invalid_safety_parameters_fail(monkeypatch, name, value):
    with pytest.raises(ValueError):
        safety_node(monkeypatch, {name: value})


def test_safety_clock_timeout_clamping_and_idle_heartbeat(monkeypatch):
    from geometry_msgs.msg import Twist
    from rclpy.clock import ClockType
    from rclpy.time import Time
    node, timer = safety_node(monkeypatch)
    assert timer.call_args.kwargs['clock'].clock_type == ClockType.STEADY_TIME
    now = [Time(seconds=1)]
    node.safety_clock = SimpleNamespace(now=lambda: now[0])
    node.tick()
    assert node.pub.publish.call_args.args[0] == Twist()
    msg = Twist()
    msg.linear.x = 99.0
    msg.angular.z = -99.0
    node.on_cmd(msg)
    assert node.target.linear.x == 0.5
    assert node.target.angular.z == -2.0
    node.tick()
    assert node.current.linear.x == pytest.approx(0.05)
    now[0] = Time(seconds=2)
    node.tick()
    assert node.current == Twist()
    node.tick()
    assert node.pub.publish.call_count == 4


@pytest.mark.parametrize('ekf', [False, True])
def test_bringup_consistent_ekf_and_clock(ekf):
    module = load('microros_pi5_bringup/launch/bringup.launch.py')
    context = LaunchContext()
    context.launch_configurations.update({
        'params_file': 'base.yaml', 'ekf_params_file': 'ekf.yaml',
        'use_ekf': str(ekf).lower(), 'namespace': '', 'use_sim_time': 'true'})
    with patch.object(module, 'get_package_share_directory', side_effect=lambda name: str(ROOT/name)), \
            patch.object(module, 'Node', side_effect=lambda **kwargs: kwargs), \
            patch.object(module, 'GroupAction', side_effect=lambda actions: actions):
        actions = module._setup(context)[0]
    nodes = {a['name']: a for a in actions if isinstance(a, dict)}
    for node in nodes.values():
        assert {'use_sim_time': True} in node['parameters']
    odom = nodes['odom_republisher']['parameters'][-1]
    assert odom['publish_tf'] is not ekf
    assert odom['output_topic'] == ('odom_wheel' if ekf else 'odom')
    if ekf:
        assert nodes['ekf_filter_node']['remappings'] == [('odometry/filtered', 'odom')]
        assert 'imu_republisher' in nodes


def test_namespace_fails_before_starting_nodes():
    module = load('microros_pi5_bringup/launch/bringup.launch.py')
    context = LaunchContext()
    context.launch_configurations.update({
        'params_file': 'base.yaml', 'ekf_params_file': 'ekf.yaml',
        'use_ekf': 'false', 'namespace': 'robot1'})
    with patch.object(module, 'get_package_share_directory', return_value=str(ROOT)):
        with pytest.raises(RuntimeError, match='one robot'):
            module._setup(context)


def test_imu_repair_and_nonfinite_rejection():
    from sensor_msgs.msg import Imu
    from rclpy.time import Time
    from unittest.mock import Mock
    module = load('microros_pi5_bringup/microros_pi5_bringup/imu_republisher.py')
    state = SimpleNamespace(gyro_variance=0.03, pub=Mock(),
                            get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10)),
                            get_logger=lambda: Mock())
    msg = Imu()
    msg.angular_velocity.z = 0.5
    module.ImuRepublisher.on_imu(state, msg)
    out = state.pub.publish.call_args.args[0]
    assert out.header.stamp.sec == 10
    assert out.header.frame_id == 'imu_frame'
    assert out.angular_velocity_covariance[8] == 0.03
    assert out.orientation_covariance[0] == -1.0
    assert msg.header.stamp.sec == 0
    msg.angular_velocity.z = float('nan')
    module.ImuRepublisher.on_imu(state, msg)
    assert state.pub.publish.call_count == 1


def test_stale_wheel_odometry_stops_integration():
    from rclpy.time import Time
    from unittest.mock import Mock
    module = load('microros_pi5_bringup/microros_pi5_bringup/wheel_joint_publisher.py')
    state = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=2)),
        last_time=Time(seconds=1.9), last_odom_time=Time(seconds=1), odom_timeout=0.5,
        vx=0.5, wz=1.0, wheel_separation=0.18, wheel_radius=0.0325,
        left_angle=0.5, right_angle=0.7, publish_gimbal=True, pub=Mock())
    module.WheelJointPublisher.tick(state)
    assert (state.left_angle, state.right_angle) == (0.5, 0.7)
    assert state.left_rate == state.right_rate == 0.0


def test_mapless_parameter_cleanup_and_configuration():
    import yaml
    from launch.actions import RegisterEventHandler
    module = load('microros_pi5_nav/launch/nav2.launch.py')
    context = LaunchContext()
    context.launch_configurations.update({
        'use_sim_time': 'false', 'autostart': 'true',
        'params_file': str(ROOT/'microros_pi5_nav/config/nav2_params.yaml'),
        'slam': 'false', 'mapless': 'true', 'map': '', 'rviz_config': ''})
    generated = []
    write = module._write_mapless_params
    def capture(path):
        generated.append(write(path))
        return generated[-1]
    with patch.object(module, 'get_package_share_directory', side_effect=lambda name: str(ROOT/name)), \
            patch.object(module, 'Node', side_effect=lambda **kwargs: kwargs), \
            patch.object(module, '_write_mapless_params', side_effect=capture):
        actions = module._setup(context)
    path = Path(generated[0])
    params = yaml.safe_load(path.read_text())
    costmap = params['global_costmap']['global_costmap']['ros__parameters']
    assert costmap['global_frame'] == 'odom'
    assert 'static_layer' not in costmap['plugins']
    assert costmap['rolling_window'] is True
    registration = next(a for a in actions if isinstance(a, RegisterEventHandler))
    from launch.events import Shutdown
    for action in registration.event_handler.handle(Shutdown(reason='test'), context):
        action.execute(context)
    assert not path.exists()


def test_nonfinite_odometry_is_not_published():
    from nav_msgs.msg import Odometry
    from unittest.mock import Mock
    module = load('microros_pi5_bringup/microros_pi5_bringup/odom_republisher.py')
    state = SimpleNamespace(pub=Mock(), get_logger=lambda: Mock())
    msg = Odometry()
    msg.pose.pose.position.x = float('nan')
    module.OdomRepublisher.on_odom(state, msg)
    state.pub.publish.assert_not_called()
