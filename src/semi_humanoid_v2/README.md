# semi_humanoid_v2

ANTBot swerve base with an OpenArm v2.0 bimanual (pinch grippers) on a
**prismatic lift column**, plus a closed cargo box behind the column. Runs in
Ignition Gazebo Fortress via `ign_ros2_control`.

Derived from `antbot_openarm_description` (same base, sensors, Gazebo fixes);
the layout below matches the Blender model `semi_humanoi_v1.blend`.

## Layout (base_link frame, metres; ground is z ≈ 0.017)

| Part | Value |
|---|---|
| Column foot (`openarm_body_mount_joint`) | `0.10 0 0.332` |
| Rail (`openarm_body_link0`) | 60×60, up to 0.75 above the foot (z 1.082) |
| Lift (`openarm_lift_joint`, prismatic z) | origin 0.50 above the foot; q ∈ [−0.34, 0] |
| Shoulders (`openarm_lift_link` frame) | y = ±0.186, on the lift line |
| Shoulder bar (split around the rail) | 40×40, \|y\| 0.07 … 0.196, on the lift |
| Top bar (`openarm_top_crossbar_link`, fixed to rail) | 40×40 × 0.746 long, 0.60 above the foot (z 0.932) |
| Cargo box | x −0.355 … 0.000, y ±0.26, z 0.333 … 0.783, notched around the column foot |

Lift positions:

| `openarm_lift_joint` | shoulder height | gripper tip, arm straight down |
|---|---|---|
| 0.00 (home, spawn) | 0.832 m | 0.226 m |
| −0.226 | 0.606 m | ≈ 0 (touches the ground) |
| −0.34 (bottom stop) | 0.492 m | −0.114 m (bend the arm first) |

Upward travel ends at q = 0: above that the chest hits the top bar.

## semi_humanoid_v3

`urdf/semi_humanoid_v3.urdf.xacro` (+ `openarm_v3_lift.xacro`, `cargo_box_v3.xacro`,
`meshes/openarm_body_column_v3.stl`); `urdf/semi_humanoid_v3.urdf` is the
pre-expanded copy. Launch with `model:=semi_humanoid_v3`. All z below are
height above ground: `base_link` lies on the ground plane (wheel axle at z = 0.103).

| Part | v3 value |
|---|---|
| Column foot (`openarm_body_mount_joint`) | `0.10 0 0.315`; plate 260×220×12 at base_link x 0.01 … 0.27, rail clamp 100×100 |
| Rail | 60×60, top at z 1.065 |
| Lift (`openarm_lift_joint`) | q ∈ [−0.34, +0.16] (0.50 m stroke), no top bar |
| Shoulders | y = ±0.235: straight arms clear the chassis side by 5 cm over the whole stroke |
| Shoulder bar (split around the rail) | 40×40, \|y\| 0.07 … 0.245 |
| Cargo box | x −0.355 … 0.000, y ±0.26, z 0.316 … 0.766, no notch |
| S10 side cameras | `s10_ring_openarm.xacro`: (0, ±0.35, 0.55), 40° down; nearest arm link 11.6 mm at q ≈ −0.29 … −0.19 |
| 2D LiDARs | real, inverted mounts at (±0.325, 0, 0.238); `lidar_2d_*_extrinsic` in the calibration yaml is honoured (unlike v2) |
| Optional 3D LiDARs | `airy:=true` / `airy_back:=true` (RoboSense Airy pair), `airy_vertical_samples:=192\|96\|48`; off by default. `gazebo.launch.py` forwards them only with `model:=semi_humanoid_v3` |

| `openarm_lift_joint` | shoulder height | gripper tip, arm straight down |
|---|---|---|
| +0.16 (top) | 0.975 m | 0.369 m |
| 0.00 (home, spawn) | 0.815 m | 0.209 m |
| −0.209 | 0.606 m | ≈ 0 (touches the ground) |
| −0.34 (bottom stop) | 0.475 m | −0.131 m (bend the arm first) |

## Files

- `urdf/semi_humanoid_v2.urdf.xacro` – main description (args: `camera`, `side_cameras`, `sim_gazebo`, `calibration_yaml_path`)
- `urdf/semi_humanoid_v2_real.urdf.xacro` – real-robot sensor layout (args: `camera`, `side_cameras`, `calibration_yaml_path`); used by `robot_sensor_gazebo.launch.py`
- `urdf/semi_humanoid_v3.urdf.xacro` – v3 body + the real sensor layout and the Airy pair (args: `camera`, `side_cameras`, `sim_gazebo`, `airy`, `airy_back`, `airy_vertical_samples`, `calibration_yaml_path`); `gazebo.launch.py model:=semi_humanoid_v3`
- `urdf/airy_lidar.xacro` – the `AiryLidar` macro, shared by `semi_humanoid_v2_airy.urdf.xacro` and `semi_humanoid_v3.urdf.xacro`
- `urdf/openarm_v2_lift.xacro` – column, lift joint, chest, both bars and the two arms
- `urdf/cargo_box.xacro`, `urdf/s10_ring_openarm.xacro`, `urdf/ros2_control_gazebo.xacro`, `urdf/gazebo_plugins.xacro`
- `config/controllers_gazebo.yaml` – swerve + `lift_controller` + `left/right_arm_controller` + `left/right_gripper_controller` (JointTrajectoryController, position)
- `meshes/openarm_body_column.stl` (foot plate + rail), `meshes/openarm_lift_chest.stl` (chest, in the lift frame), `meshes/base_chassis.stl`, `meshes/openarm/` (OpenArm meshes; `collision/mirrored/` = left-arm collisions with the Y flip baked in, which DART needs)

## Run

```bash
colcon build --packages-select semi_humanoid_v2
source install/setup.bash
ros2 launch semi_humanoid_v2 gazebo.launch.py
```

Options: `world:=depot`, `headless:=true`, `camera:=true`, `side_cameras:=true`,
`model:=semi_humanoid_v3`, `calibration_yaml_path:=...`. With
`model:=semi_humanoid_v3` also `airy:=true`, `airy_back:=true` and
`airy_vertical_samples:=48`.

Lower the shoulders until straight arms reach the ground, then back home:

```bash
ros2 action send_goal /lift_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [openarm_lift_joint], points: [{positions: [-0.226], time_from_start: {sec: 4}}]}}"
ros2 action send_goal /lift_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [openarm_lift_joint], points: [{positions: [0.0], time_from_start: {sec: 4}}]}}"
```

## Real-robot model in simulation

`launch/robot_sensor_gazebo.launch.py` spawns `urdf/semi_humanoid_v2_real.urdf.xacro` (same body, real upside-down 2D LiDAR mounts, calibration support) with the same sensors as `antbot_openarm_description/robot_sensor_gazebo.launch.py`:

| arg (default) | sensor | ROS topic |
|---|---|---|
| always | 2D LiDAR front / back | `/scan_0`, `/scan_1` |
| always | IMU | `/imu/data` |
| `camera` (false) | S10 front | `/sensor/camera/stereo_front/*` |
| `side_cameras` (false) | S10 back / left / right | `/sensor/camera/stereo_<position>/*` |

```bash
ros2 launch semi_humanoid_v2 robot_sensor_gazebo.launch.py camera:=true side_cameras:=true world:=depot
```

Also `headless:=true`, `calibration_yaml_path:=...`. Controllers are the same as `gazebo.launch.py`, lift included. The real-hardware ros2_control block (`real_robot.launch.py`) is not ported yet.

## Simulation notes

- In Gazebo every OpenArm joint stop is 0.02 rad outside its real limit, and the lift stops 5 mm outside [−0.34, 0]. DART stops answering velocity commands (how `gz_ros2_control` applies position commands) once a joint touches its limit ([gz-sim#1684](https://github.com/gazebosim/gz-sim/issues/1684)).
- The lift moves at most 0.10 m/s, so give trajectories enough time (the full 0.34 m needs ≥ 3.4 s).
- Gazebo model self-collision is off (SDF default), so the arms pass through the robot's own base and cargo box; they still collide with the ground and other models. Plan motions accordingly.
