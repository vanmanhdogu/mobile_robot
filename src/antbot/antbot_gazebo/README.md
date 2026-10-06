# antbot_gazebo

Ignition Gazebo simulation package for AntBot swerve-drive robot.

## Prerequisites

```bash
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-xacro ros-humble-robot-state-publisher
```

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to antbot_gazebo antbot_teleop
source install/setup.bash
```

## Quick Start

**Terminal 1** — Launch Gazebo simulation:
```bash
ros2 launch antbot_gazebo gazebo.launch.py
```

**Terminal 2** — Keyboard teleop:
```bash
ros2 run antbot_teleop teleop_keyboard
```

Specify a world by name (resolved via `config/worlds.yaml`) or full path:
```bash
ros2 launch antbot_gazebo gazebo.launch.py world:=depot
ros2 launch antbot_gazebo gazebo.launch.py world:=/path/to/world.sdf
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `empty` | World name (resolved via `config/worlds.yaml`) or full path to SDF file |
| `headless` | `false` | Run the server without the GUI (`ign gazebo -s`) |
| `lidar_3d` | `false` | Simulate the 3D LiDAR → `/lidar_3d_points` |
| `camera` | `false` | Simulate the front RGBD camera → `/sensor/camera/stereo_front/*` |
| `side_cameras` | `false` | Simulate the left/right/back RGBD cameras → `/sensor/camera/stereo_<position>/*` |
| `mono_cameras` | `false` | Simulate the 4 mono cameras → `/sensor/camera/v4l2_driver/<position>/*` |

### Optional sensors

The 3D LiDAR and the cameras are **off by default**: they are rendering
sensors and each one costs GPU time every frame.

```bash
# 3D LiDAR + front RGBD camera (measured RTF ~1.0 on Jetson Orin, empty world)
ros2 launch antbot_gazebo gazebo.launch.py world:=depot lidar_3d:=true camera:=true

# 360 deg depth coverage: all four S10 RGBD cameras. Each is one more render
# pass, but at 240 x 160 they are ~7x cheaper per pass than a mono camera.
ros2 launch antbot_gazebo gazebo.launch.py world:=depot \
  lidar_3d:=true camera:=true side_cameras:=true

# Everything, including the 4 mono cameras (measured RTF ~0.3 — sim time
# runs at a third of wall clock, Nav2 still works but tuning numbers shift)
ros2 launch antbot_gazebo gazebo.launch.py world:=depot \
  lidar_3d:=true camera:=true side_cameras:=true mono_cameras:=true
```

Sim topic names match the real drivers, so nothing downstream needs
sim-specific topic configuration:

| Sim topic | Real source |
|-----------|-------------|
| `/lidar_3d_points` | `vanjee_lidar_sdk` (WLR-722), same frame `lidar_3d_link` |
| `/sensor/camera/v4l2_driver/<position>/image_raw` | `antbot_camera` node |
| `/sensor/camera/stereo_front/{image_raw,depth/image_raw,points,camera_info}` | front S10 RGBD camera |
| `/sensor/camera/stereo_{left,right,back}/{image_raw,depth/image_raw,points,camera_info}` | side and rear S10 RGBD cameras |

Verified frames and directions (probe world with one coloured wall per axis):

| Topic | frame_id | Looks toward |
|-------|----------|--------------|
| `/sensor/camera/stereo_front/*` | `camera_stereo_front_depth_link` | +X, pitched 30 deg down |
| `/sensor/camera/stereo_left/*` | `camera_stereo_left_depth_link` | +Y, pitched 30 deg down † |
| `/sensor/camera/stereo_right/*` | `camera_stereo_right_depth_link` | -Y, pitched 30 deg down † |
| `/sensor/camera/stereo_back/*` | `camera_stereo_back_depth_link` | -X, pitched 30 deg down † |
| `/sensor/camera/v4l2_driver/front/*` | `camera_mono_front_optical_link` | +X |
| `/sensor/camera/v4l2_driver/left/*` | `camera_mono_left_optical_link` | +Y |
| `/sensor/camera/v4l2_driver/right/*` | `camera_mono_right_optical_link` | -Y |
| `/sensor/camera/v4l2_driver/back/*` | `camera_mono_back_optical_link` | -X |

† The side and rear RGBD mounts are **nominal bracket positions**, not measured
extrinsics, and their directions have not been probed in a coloured-wall world
yet. Replace the `xyz`/`rpy` defaults in `urdf/antbot_sim.xacro` (and the
matching `CalibratedSensors` entries in `antbot_description/urdf/antbot.xacro`)
once the hardware is mounted and calibrated. At the nominal 0.50 m height and
30 deg tilt, each side camera's lowest ray reaches the ground ~0.29 m outboard
of the body edge, leaving a blind strip that close-range obstacle checks still
have to get from `/scan_0` and `/scan_1`; tilt further down if you need the
cameras to cover it.

## Package Structure

```
antbot_gazebo/
├── config/
│   ├── swerve_controller_gazebo.yaml   # Sim-specific controller params
│   └── worlds.yaml                     # World name → SDF path mapping
├── launch/
│   └── gazebo.launch.py                # Gazebo + robot spawn + controllers
├── urdf/
│   ├── antbot_sim.xacro                # Top-level sim URDF
│   ├── gazebo_plugins.xacro            # Sensor plugins (LiDAR, IMU, cameras)
│   └── ros2_control_gazebo.xacro       # IgnitionSystem hardware interface
└── worlds/
    ├── empty.sdf                       # Empty world (default)
    └── depot.sdf                       # Warehouse-style world
```

## Simulation vs Real Hardware

| Aspect | Real HW | Gazebo |
|--------|---------|--------|
| Hardware interface | BoardInterface | **IgnitionSystem** |
| Acceleration command | Supported | **Not supported** (velocity/position only) |
| 2D LiDAR | 2D LiDAR sensor | **gpu_lidar** (ogre2 required) |
| IMU | antbot_imu node | **Gazebo IMU plugin** |
| Control rate | 20 Hz | **100 Hz** |
| Scrub compensation | Required | **Not needed** |
| IK iterations | 0 | **3** (55mm offset correction) |
| Odom integration | rk4 | **analytic_swerve** |
| Odom smoothing | window: 1 | **window: 10** |

## Launch Sequence

```
gazebo.launch.py
  ├── Environment variables (IGN_GAZEBO_RESOURCE_PATH, PLUGIN_PATH)
  ├── xacro → URDF generation
  ├── config/worlds.yaml → resolve world name to SDF path
  ├── ign gazebo -r world.sdf
  ├── Robot spawn (x=0, y=0, z=0.15)
  ├── robot_state_publisher
  ├── [spawn_robot exit wait (OnProcessExit)]
  ├── controller_manager service wait
  ├── joint_state_broadcaster spawn
  ├── swerve_drive_controller spawn (after JSB completes)
  └── ros_gz_bridge (sensor topic bridging)
```

## URDF Structure

```
antbot_sim.xacro (simulation entry point)
  ├── antbot_description/  (shared — same as real HW)
  │    ├── sensors.xacro, base.xacro, wheel.xacro
  └── antbot_gazebo/  (sim-specific)
       ├── ros2_control_gazebo.xacro   IgnitionSystem
       └── gazebo_plugins.xacro        friction + sensors
```

### Hardware Interface

```
IgnitionSystem (ign_ros2_control)
  ├── Wheels (x4):    velocity cmd → velocity/position state
  └── Steering (x4):  position cmd → position/velocity state
```

> **Note**: IgnitionSystem does not support `acceleration` or `effort` command interfaces.
> Controller config must set `use_acceleration_command: false`.

### Friction Model

| Part | Friction coefficient | Description |
|------|---------------------|-------------|
| base_link | 0.2 | Low friction (sliding) |
| wheel (x4) | 1.8 | High friction (traction) |
| steering (x4) | 0.0 | No friction (free rotation) |

## Simulated Sensors

| Sensor | Plugin | Topic | Samples | Range | Noise |
|--------|--------|-------|---------|-------|-------|
| Front 2D LiDAR | `gpu_lidar` | `/scan_0` | 720 | 0.6-20m | stddev 0.008 m |
| Back 2D LiDAR | `gpu_lidar` | `/scan_1` | 720 | 0.6-20m | stddev 0.008 m |
| IMU | `imu_sensor` | `/imu/data` | — | — | angular 0.0003 rad/s, linear 0.02 m/s² |
| 3D LiDAR *(`lidar_3d:=true`)* | `gpu_lidar` | `/lidar_3d_points` | 900 x 32, 10 Hz | 0.1-70m | stddev 0.01 m |
| Front RGBD *(`camera:=true`)* | `rgbd_camera` | `/sensor/camera/stereo_front/*` | 240x160, 15 Hz | 0.3-8m | stddev 0.01 m |
| 3x Side/rear RGBD *(`side_cameras:=true`)* | `rgbd_camera` | `/sensor/camera/stereo_{left,right,back}/*` | 240x160, 15 Hz | 0.3-8m | stddev 0.01 m |
| 4x Mono *(`mono_cameras:=true`)* | `camera` | `/sensor/camera/v4l2_driver/<position>/*` | 640x360, 15 Hz | 0.1-50m | stddev 0.005 |

> **RGBD cameras model the S10 ToF module**: 240 x 160 depth at 15 fps, 90° x
> 60° FOV, 0.3–8 m against 90% reflectivity. Two datasheet lines do not carry
> over to Gazebo. First, the S10's 1920 x 1080 RGB stream: a gz `rgbd_camera`
> renders one image buffer and derives both colour and depth from it, so the
> colour image comes out at the ToF resolution; put a `MonoCameraSensor` on the
> same link if a task needs full-resolution colour. Second, the 0.3–3 m low
> reflectivity range — gz depth is reflectivity-independent, so every surface
> is read out to 8 m. For the 120° x 80° variant pass `hfov="${radians(120)}"`
> to `RgbdCameraSensor`.

> **3D LiDAR geometry** is an approximation of the Vanjee WLR-722
> (32 rings, -25°..+15° vertical). Tune the `Lidar3DSensor` macro defaults in
> `urdf/gazebo_plugins.xacro` if you need the exact ring layout.

> **gpu_lidar publishes two topics**: `ignition.msgs.LaserScan` on `<topic>`
> and `ignition.msgs.PointCloudPacked` on `<topic>/points`. Only the point
> cloud is bridged for the 3D LiDAR — one flattened ring of a 3D scan is not
> useful, and Nav2 already has `/scan_0` and `/scan_1`.

> **Camera sensors must mount on the x-forward `*_link`, never on the
> `*_optical_link`.** A Gazebo camera looks down its mount link's **+X** axis.
> An optical frame's +X points to the right and its +Z is what points forward,
> so mounting a camera there aims it 90 deg off to the side. The macros take a
> separate `frame` parameter for the frame_id to report, so the mount link and
> the advertised frame can differ.

> **Point cloud convention**: gz-sensors publishes the RGBD point cloud in the
> camera's own x-forward frame, *not* in the REP-103 optical convention. That
> is why `camera_stereo_front` reports `camera_stereo_front_depth_link` rather
> than the optical frame — the cloud and its frame_id have to agree or the
> cloud lands rotated in RViz. The mono cameras publish images only, so they
> report the `*_optical_link` that `camera_info` consumers expect.

> **Note**: `gpu_lidar` requires the **ogre2** render engine. Set `<render_engine>ogre2</render_engine>` in your world SDF.

## Controller Configuration

Config file: `config/swerve_controller_gazebo.yaml`

| Parameter | Sim value | HW value | Reason |
|-----------|-----------|----------|--------|
| `non_coaxial_ik_iterations` | **3** | 0 | Gazebo exposes 55mm offset error |
| `enable_steering_scrub_compensator` | **false** | true | No real scrub in Gazebo |
| `velocity_rolling_window_size` | **10** | 1 | Smooth sim encoder noise |
| `odom_integration_method` | **analytic_swerve** | rk4 | Exact for piecewise-constant |
| `use_acceleration_command` | **false** | true | IgnitionSystem limitation |

## Custom Worlds

### worlds.yaml

Simulation (SDF) and navigation (map) use separate `worlds.yaml` files:

| File | Purpose | Mapping |
|------|---------|---------|
| `antbot_gazebo/config/worlds.yaml` | Gazebo simulation | World name → SDF file |
| `antbot_navigation/maps/worlds.yaml` | Nav2 navigation | World name → map file |

```yaml
# antbot_gazebo/config/worlds.yaml
worlds:
  empty:
    sdf: empty.sdf
  depot:
    sdf: depot.sdf        # relative to antbot_gazebo/worlds/
```

### Registering a New World

1. Place the SDF file in `antbot_gazebo/worlds/`
2. Add an entry to `antbot_gazebo/config/worlds.yaml`
3. If Nav2 navigation is needed:
   - Build a map with SLAM: `ros2 launch antbot_navigation slam.launch.py mode:=sim`
   - Save it: `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_world`
   - Copy `.pgm` and `.yaml` to `antbot_navigation/maps/`
   - Register in `antbot_navigation/maps/worlds.yaml`

### World SDF Requirements

- `render_engine: ogre2` (required for gpu_lidar)
- Physics, Sensors, UserCommands, SceneBroadcaster plugins
- ground_plane + sun (lighting)

## Troubleshooting

- **gpu_lidar returns only range_min** — Check that `render_engine` is `ogre2`. Using `ogre` causes all rays to return minimum values.
- **Controller activation failure** — Run `ros2 control list_controllers`. If stuck in `unconfigured`, check Gazebo logs for `gz_ros2_control` plugin errors.
- **No sensor topics at all** — The world SDF is missing the `Sensors` (and for the IMU, `Imu`) system plugin. Both `empty.sdf` and `depot.sdf` carry them; a custom world needs them too.
- **A camera looks in the wrong direction** — Check which link the sensor is mounted on. It must be the x-forward `*_link`; an `*_optical_link` mount rotates the camera 90 deg. Probe it by putting a differently coloured wall on each side of the robot and checking which colour dominates the image.
- **Real-time factor collapses when cameras are on** — Each rendering sensor is a render pass per frame. `lidar_3d:=true camera:=true` holds RTF ~1.0; adding `mono_cameras:=true` drops it to ~0.3. Enable only the sensors the task needs.
- **Odometry drift** — `non_coaxial_ik_iterations` set to 0 causes drift from the 55mm steering-wheel offset. Set to 2-3 in simulation.
