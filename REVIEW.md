# Repository review — 2026-10-22

The repository boundary is `src/microros_pi5`, containing four ROS 2 packages.
Keep all four. Optional at runtime does not mean disposable source code.

## What belongs in Git

| Content | Priority | Decision |
| --- | --- | --- |
| `microros_pi5_description/urdf`, launch files | Core | Keep: robot geometry and TF tree |
| `microros_pi5_bringup` nodes, launch and config | Core | Keep: odometry, joint states, velocity limits and teleop |
| `microros_pi5_nav` launch and config | Core for autonomous driving | Keep: SLAM, saved-map and mapless navigation |
| `microros_pi5_sim` | Development | Keep: hardware-free odometry and control debugging; no lidar or collision simulation |
| RViz configurations | Useful | Keep: reproducible views for each operating mode |
| `ekf.yaml`, optional agent launch, `scan_qos_relay.py` | Optional runtime features | Keep: EKF needs validation; agent launch is for native installations; relay only for a demonstrated QoS mismatch |
| `package.xml`, `setup.py`, `setup.cfg`, resource markers, `__init__.py`, `CMakeLists.txt` | Packaging | Keep, including empty files: ROS discovery and installation depend on them |
| `maps/.gitkeep`, `meshes/README.md` | Small placeholders | Keep; they document intended asset locations |
| Curated map YAML plus referenced image | Optional assets | Commit together when intentionally sharing a map; current maps directory is empty |
| README, LICENSE, this review, regression tests | Maintenance | Keep |
| `__pycache__`, `.pyc`, egg-info, build/install/log output | Generated | Ignore; retain locally if useful |
| Bags, local maps, editor state and `.env` files | Local data | Ignore by default |

Workspace-level `docs/` and `scripts/` are outside this repository. In particular,
`docs/04_offline_wifi_nav.md` contains site-specific network and recovery details.
Keep that operational runbook locally; do not silently copy it into a reusable package.
The existing `board_connect.sh`, `drive.sh`, and `odom_calibrate.py` helpers are not
included. The package README identifies those external dependencies where relevant.

## Findings fixed in this preparation

1. **High — recovery motions bypassed the safety node.** Only `controller_server`
   remapped `cmd_vel`; `behavior_server` could publish recovery motion directly to
   the board. Both now route commands through `cmd_vel_raw`.
2. **High — advertised stop key drove backwards.** The keyboard banner and comments
   described `k` as stop, but its binding selected reverse. `k` now stops, and comma
   reverses, consistent with the displayed layout. `s` and down-arrow still reverse.
3. **Packaging — missing direct runtime dependencies.** Added the launch package
   index dependency, headless joint-state publisher, and Nav2/Python dependencies
   used directly by this source. Removed unused `tf_transformations` dependency.
4. **Documentation — misleading stop and calibration instructions.** Clarified that
   zero commands still pass through the deceleration ramp, and that a host watchdog
   cannot protect against loss of its own connection to the board. Removed instructions
   to add a second lidar driver or change visualization wheel geometry to calibrate
   board odometry. Simulation examples now use the included keyboard node.
5. **Git hygiene.** Added ignore rules and the license text matching the existing MIT
   declarations. Removed the README's link outside the repository and hardcoded SSH
   destination from its launch example.

## Follow-up fixes

- Reject non-finite velocity commands and invalid safety limits/rates; use a steady-clock
  watchdog and continuously publish zero heartbeats at rest.
- Route repaired wheel odometry to `odom_wheel` in EKF mode and expose the EKF output
  as `odom`. A new best-effort-to-reliable IMU bridge supplies locally stamped gyro
  data with fallback variance; fuse yaw rate only until the IMU is calibrated.
- Forward simulation time into bringup and robot-state publishing. Safety timeout remains
  independent of simulation time.
- Reject unsupported nonempty namespaces instead of silently producing a partly isolated
  robot graph. Separate robot domains are the supported isolation strategy.
- Stop wheel visualization on stale odometry; reject non-finite odometry pose/twist.
- Lower DWB's stopped threshold below its maximum driving speed.
- Install PNG map images, clean up mapless parameter files on orderly launch shutdown,
  default the native agent to the observed serial setup, and reject resolved scan-relay loops.

## Hardware work still required

Firmware is not present, so a board-side watchdog cannot be implemented here. Verify it and
physical stopping before unattended motion. Measure geometry/footprint, calibrate firmware
odometry scale, and validate EKF covariance, gyro axes and Nav2 tuning on the actual robot.
Gimbal positions are visualization placeholders. Simulation has no lidar or collision physics.
Hard process termination can still leave temporary launch files; orderly shutdown cleans them.
These are explicit deployment limits, not claims of completed hardware validation.

## Validation and handoff

Regression tests exercise the keyboard stop/reverse behavior and generated Nav2 nodes
in all three modes, without starting ROS nodes or publishing motion commands:

```bash
source /opt/ros/humble/setup.bash
ROS_LOG_DIR=/tmp/microros-review-ros-log python3 -m pytest -q tests
```

All 22 offline regression checks passed. An isolated ROS 2 Humble colcon build completed
for all four packages (build/install/log output under `/tmp/microros-final-*`;
CMake lint testing was disabled for that build).
Python syntax, package XML, YAML and RViz configuration parsing were checked. The xacro
expanded successfully and `check_urdf` accepted the resulting tree. No hardware, WiFi,
EKF, or end-to-end navigation test was performed in this review. Historical tests in
README remain historical, not a claim about the new changes.

For a fresh checkout in `~/ros2_ws/src/microros_pi5`, install dependencies and build:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/microros_pi5 --ignore-src -r -y
colcon build --packages-select microros_pi5_description microros_pi5_bringup \
  microros_pi5_sim microros_pi5_nav --symlink-install
source install/setup.bash
```

The external micro-ROS agent is installed separately, or run using your existing
container setup. This preparation does not install system dependencies.

Target repository: `pacharawit-gan/microros` (private). The initial commit uses only
the owner’s author/committer identity, with no co-author trailer. GitHub determines
the displayed contributor list from account-linked commit identities.

For subsequent changes:

```bash
cd ~/ros2_ws/src/microros_pi5
git diff --cached --stat
git diff --cached --check
git commit -m "Describe the change"
git push -u origin main
```
