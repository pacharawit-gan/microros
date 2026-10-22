# MicroROS Pi5

ROS 2 Humble packages for the Yahboom MicroROS-Pi5 car, maintained by
[pacharawit-gan](https://github.com/pacharawit-gan).

| Package | Purpose | Needed for |
| --- | --- | --- |
| `microros_pi5_description` | URDF, TF geometry, RViz model views | Robot bringup |
| `microros_pi5_bringup` | Odometry repair, gyro preprocessing, joint states, velocity safety and teleop | Real robot and simulation |
| `microros_pi5_nav` | SLAM, saved-map and mapless Nav2 launch/configuration | Autonomous navigation |
| `microros_pi5_sim` | Kinematic fake control board | Development without hardware |

Keep all four in Git. See [REVIEW.md](REVIEW.md) for the inventory, changes and validation.
This is a development stack: physical dimensions, odometry calibration and navigation tuning
still require validation on the actual robot.

## Build

Place this repository at `~/ros2_ws/src/microros`:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/microros --ignore-src -r -y
colcon build --packages-select microros_pi5_description microros_pi5_bringup \
  microros_pi5_sim microros_pi5_nav --symlink-install
source install/setup.bash
```

If your existing checkout is named `src/microros_pi5`, use that path for `rosdep` instead.
The ROS package names are independent of the checkout directory name.
The micro-ROS agent is external: install it natively or use your existing Pi container setup.
Workspace-level WiFi runbooks and board reset/calibration scripts are not bundled here.

## Run without hardware

```bash
ros2 launch microros_pi5_description display.launch.py
# Or use the kinematic fake board:
ros2 launch microros_pi5_sim sim.launch.py
# In another sourced terminal:
ros2 run microros_pi5_bringup keyboard_teleop
```

`display.launch.py gui:=false` uses the headless joint-state publisher.
Simulation generates odometry and IMU, but has no lidar, collisions or wheel slip model.
It cannot validate SLAM/navigation or demonstrate physical safety. It uses wall time,
so it does not need `/clock`.

## Real robot

The existing board setup uses domain 20 and Fast DDS. Set these in every robot terminal:

```bash
export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Start exactly one micro-ROS agent on the Pi. For a native installation:

```bash
ros2 launch microros_pi5_bringup micro_ros_agent.launch.py
```

The default transport is serial, `/dev/ttyUSB0`, 921600 baud. For firmware explicitly
configured for UDP, use `transport:=udp4 port:=8090`. Do not start a native agent alongside
an existing container agent on the same port.

Then on the machine running this stack:

```bash
ros2 launch microros_pi5_bringup bringup.launch.py
# Separate terminal; keyboard input requires ros2 run:
ros2 run microros_pi5_bringup keyboard_teleop
```

Stop the vendor joy stack first. Only `cmd_vel_safety` should publish the board's `cmd_vel`.
Do not run teleop and Nav2 simultaneously: they share `cmd_vel_raw` without arbitration.
Do not start separate bringup when the navigation launch already includes it.

Keyboard: `i`/`w`/up drives forward, comma/`s`/down reverses, `j`/`a` and `l`/`d` turn.
`k` or space requests zero on the next teleop tick. Key release waits up to 0.75 seconds
before requesting zero. The safety node then applies its deceleration limit; this is not
an emergency stop.

## Command and sensor paths

```text
teleop / Nav2 controller / Nav2 recovery
                 |
             cmd_vel_raw
                 |
           cmd_vel_safety -------- cmd_vel --------> ESP32

ESP32 odom_raw -> odom_republisher -> odom + odom-to-base_footprint TF
ESP32 odom_raw -> wheel_joint_publisher -> joint_states
```

The safety node rejects non-finite commands, clamps speed, limits acceleration, and ramps
to zero after 0.5 seconds without valid input. Its timer and timeout use a steady clock,
so a paused or reset `/clock` cannot suspend the watchdog. It publishes zero heartbeats
at rest, including startup and after reconnection.

**A host watchdog cannot stop a disconnected board or survive its own process being killed.**
A board-side command timeout and physical stop behavior must be verified before unattended
operation. Firmware source is not included in this repository.

The odometry republisher restamps incoming odometry, fills missing covariance, rejects
non-finite pose/twist values, and normalizes orientation. It does not calibrate distance.
Wheel joint angles are estimated visualization only, stop advancing when input becomes
stale, and are not encoder feedback. Gimbal angles remain fixed at zero.

### Optional EKF

```bash
ros2 launch microros_pi5_bringup bringup.launch.py use_ekf:=true
```

This switches to a single consistent fused output:

```text
odom_raw -> odom_republisher -> odom_wheel ----+
                                              +-> EKF -> odom + TF
imu -> imu_republisher -> imu_sanitized -------+
```

The odometry republisher disables its TF output in this mode. The IMU bridge accepts
best-effort board data, restamps it locally, supplies missing gyro variance, and republishes
reliably. Only gyro yaw rate is fused from the IMU; orientation and acceleration are excluded
until calibrated. Verify gyro axis, covariance, latency and motion response on hardware
before relying on the filter. Local restamping does not correct network latency.

### Frames and isolation

The main chain is `map -> odom -> base_footprint -> base_link`. The robot description supplies
`imu_frame` and `laser_frame`, matching the observed board frame IDs. SLAM or AMCL owns
`map -> odom`; the republisher or EKF owns `odom -> base_footprint`, never both.

This stack supports one robot per ROS domain. A nonempty `namespace` argument is rejected
because node namespaces alone do not isolate TF frames. Use separate `ROS_DOMAIN_ID` values
with matching firmware/agent settings for separate robots.

For bag replay, pass `use_sim_time:=true` to SLAM, navigation or bringup and provide `/clock`.
Included bringup and robot-state publishing receive the same clock. The command safety
watchdog intentionally continues to use steady time. Keep replay isolated from real hardware.

## Mapping and navigation

```bash
# Mapping (includes bringup):
ros2 launch microros_pi5_nav slam.launch.py
# Drive with keyboard_teleop in a separate terminal.

# Save outside the source tree, or intentionally store a curated map in the package:
ros2 run nav2_map_server map_saver_cli -f /absolute/path/my_map

# Navigate a saved map (includes bringup):
ros2 launch microros_pi5_nav nav2.launch.py map:=/absolute/path/my_map.yaml
```

Use RViz **2D Pose Estimate** for AMCL initialization, then **2D Goal Pose**.
Package maps can be selected by bare name (`map:=my_map`) after rebuilding. Commit the YAML
and its referenced PGM/PNG together only when the map is intended to be shared.

```bash
# Navigate while the mapping launch is already running:
ros2 launch microros_pi5_nav nav2.launch.py slam:=true bringup:=false

# Or navigate locally without building a map:
ros2 launch microros_pi5_nav nav2.launch.py mapless:=true
```

Mapless mode uses `odom`, a 10-by-10-m rolling global costmap, and live lidar. Goals must be
inside that window (roughly 5 m along each axis from the robot). It forgets obstacles outside
the window and never corrects odometry drift. Start with close goals and validate with wheels
lifted. `slam:=true` and `mapless:=true` are mutually exclusive.

Both controller and recovery motion pass through `cmd_vel_safety`. Nav2's additional velocity
smoother is omitted. Keep Nav2 velocity limits below the safety limits. The stopped-velocity
threshold is 0.01 m/s, below the configured maximum 0.22 m/s; tune it against real sensor noise.

The MS200 scan comes through the ESP32; no second lidar driver is needed. The board's sensor
publishers use best-effort QoS. Inspect them with:

```bash
ros2 topic echo /scan --qos-reliability best_effort --once --no-arr
ros2 topic info /scan --verbose
```

`scan_qos_relay` is optional for a consumer that requires reliable input. Point that consumer
to `scan_reliable`; the relay rejects input/output names that resolve to the same topic.

## Tests

From this repository, after sourcing ROS Humble and installing pytest:

```bash
ROS_LOG_DIR=/tmp/microros-tests python3 -m pytest -q tests
```

These tests do not connect to the robot. See [REVIEW.md](REVIEW.md) for the validation scope.

## Ownership and license

Maintainer: [pacharawit-gan](https://github.com/pacharawit-gan). MIT license; see [LICENSE](LICENSE).
Git commits for this initial repository use the owner's GitHub identity without co-author trailers.
ROS, Nav2, micro-ROS and vendor firmware remain external dependencies with their own authors.
