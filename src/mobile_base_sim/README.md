# mobile_base_sim

Minimal Ignition Gazebo simulation of the AntBot **mobile base**: the body, the
four swerve wheel modules, and the two 2D LiDARs. Nothing else.

| kept | dropped |
|:--|:--|
| `base_link` body (mesh + inertia) | all 5 cameras |
| 4 swerve modules — 4 steering + 4 wheel joints | 3D LiDAR |
| `lidar_2d_front`, `lidar_2d_back` | IMU, GNSS, magnetometer |
| swerve controller + odometry | charging coil |

The result is 11 links and 2 sensors, against 25 links and 9 sensors for the
full robot.

## Relationship to the antbot packages

**No file in the antbot meta package is modified, read-write, or shadowed by
this package.** The coupling is one-directional and read-only:

| what this package uses | from | how |
|:--|:--|:--|
| `base.xacro`, `wheel.xacro`, `sensors.xacro` | `antbot_description` | `xacro:include` |
| `base_link.stl`, `steering_link_r.stl`, `wheel_link_r.stl` | `antbot_description` | `package://` URI |
| `SwerveDriveController` | `antbot_swerve_controller` | ros2_control plugin, loaded by type name |

Everything else — the top-level URDF, the Gazebo sensor definitions, the
hardware interface, the controller config, the worlds and the launch files — is
owned here. Reusing the macros rather than copying them means upstream geometry
fixes arrive for free and the 19 MB of STL meshes are not duplicated; the cost
is that this package will not build without `antbot_description` on the path.

## Build

```bash
cd ~/ros2_humble_ws
colcon build --symlink-install --packages-select mobile_base_sim
source install/setup.bash
```

`--symlink-install` means later edits to `urdf/`, `launch/`, `config/`,
`worlds/` and `rviz/` take effect on the next launch with no rebuild.

## Run

Inside the container (`docker/run.sh shell` — the host has no `ign` binary):

```bash
# walled room with two pillars, so the LiDARs have something to see
ros2 launch mobile_base_sim gazebo.launch.py

# flat ground, plus RViz
ros2 launch mobile_base_sim gazebo.launch.py world:=empty rviz:=true

# no GUI, for SSH or CI
ros2 launch mobile_base_sim gazebo.launch.py headless:=true

# borrow the AntBot warehouse world
ros2 launch mobile_base_sim gazebo.launch.py \
  world:=$(ros2 pkg prefix antbot_gazebo)/share/antbot_gazebo/worlds/depot.sdf
```

Then drive it:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"   # forward
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.3}}"   # strafe
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"  # spin
ros2 run antbot_teleop teleop_keyboard                                       # keyboard
```

To inspect the geometry with no simulator at all — a slider per joint, so you
can sweep the steering modules by hand:

```bash
ros2 launch mobile_base_sim display.launch.py
```

## Launch arguments

| argument | default | meaning |
|:--|:--|:--|
| `world` | `box_room` | name from `config/worlds.yaml`, or an absolute SDF path |
| `headless` | `false` | server only, no GUI (`ign gazebo -s`) |
| `rviz` | `false` | also start RViz with `rviz/mobile_base.rviz` |
| `x`, `y`, `z`, `yaw` | `0`, `0`, `0.15`, `0` | spawn pose |

## Interface

| topic | type | direction |
|:--|:--|:--|
| `/cmd_vel` | `geometry_msgs/Twist` | in |
| `/odom` | `nav_msgs/Odometry` | out |
| `/scan_0` | `sensor_msgs/LaserScan` | out — front, frame `lidar_2d_front_link` |
| `/scan_1` | `sensor_msgs/LaserScan` | out — back, frame `lidar_2d_back_link` |
| `/joint_states` | `sensor_msgs/JointState` | out, 100 Hz |
| `/clock` | `rosgraph_msgs/Clock` | out |
| `/tf` | `odom` -> `base_link` | out, published by the controller |

LiDAR topics are named `/scan_0` and `/scan_1` to match the AntBot
simulation, so `antbot_navigation` and the tooling in the workspace docs can be
pointed at this base unchanged.

## Worlds

| name | contents |
|:--|:--|
| `empty` | flat ground plane |
| `box_room` | 10 x 10 m walled room, 1 m walls, two off-centre pillars |

> Any world you add **must** declare the `Sensors` system plugin with
> `<render_engine>ogre2</render_engine>`. Ignition does not load it by default,
> and without it the LiDARs never publish while the robot still drives
> perfectly — a confusing failure. Under the older `ogre` engine every
> `gpu_lidar` ray silently returns `range_min`. Copy the `<plugin>` block from
> `worlds/empty.sdf`.

## Verified behaviour

Measured headless in `box_room` on the Jetson:

| check | result |
|:--|:--|
| `ros2 control list_controllers` | `joint_state_broadcaster` active, `swerve_controller` active |
| `/scan_0`, `/scan_1` | 15.2 Hz, 720 samples, 360° FoV |
| `/joint_states` | 100 Hz |
| real-time factor | 1.0 |
| `/cmd_vel` `x=0.4` | `/odom` advances at 0.4 m/s |
| `/cmd_vel` `y=0.3` for 4 s | `/odom` y = +0.799 m — strafing works |
| scan max range | 6.91 m — the room corners are at 6.93 m |
| scan min range | 2.38 m — the pillar face is at 2.39 m |

About 250 of the 720 rays per scan come back below `range_min`. That is
correct: `gpu_lidar` renders **visual** geometry, so each LiDAR is occluded by
the body mesh it is mounted on, exactly as the real units are by the chassis.

## Layout

```
mobile_base_sim/
├── config/
│   ├── swerve_controller.yaml   # controller type, geometry, sim tuning
│   └── worlds.yaml              # world name -> SDF file
├── launch/
│   ├── gazebo.launch.py         # world + spawn + controllers + bridge
│   └── display.launch.py        # RViz + joint sliders, no simulator
├── rviz/mobile_base.rviz        # RobotModel + TF + both scans
├── urdf/
│   ├── mobile_base.xacro        # top level: which antbot macros to instantiate
│   ├── gazebo_plugins.xacro     # friction, ros2_control plugin, 2x gpu_lidar
│   └── ros2_control.xacro       # IgnitionSystem, 8 joints
└── worlds/{empty,box_room}.sdf
```

## Notes and gotchas

**Dimension properties live in the top-level xacro.** `base.xacro` and
`wheel.xacro` read `wheel_radius`, `steering_height` and `wheel_offset` out of
the enclosing scope rather than taking them as macro arguments, so
`urdf/mobile_base.xacro` must define them before instantiating the macros. They
are duplicated from `antbot_description/urdf/antbot.xacro`; if the real robot's
dimensions change, change them here too.

**The controller geometry must track the URDF.** `module_x_offsets`,
`module_y_offsets` and `steering_to_wheel_y_offsets` in
`config/swerve_controller.yaml` are derived from the xacro dimensions. Edit one
without the other and the odometry will quietly lie.

**`IGN_GAZEBO_SYSTEM_PLUGIN_PATH` must include the workspace overlay.** On
arm64 `libign_ros2_control-system.so` is built from source into
`install/gz_ros2_control/lib`, not `/opt/ros/humble/lib`. The launch file
*appends* to the variable; `docker/docker-compose.yml` sets it. Without it
Gazebo starts, the robot spawns, and no controller ever loads.

**`Wheel acceleration command interface not found for module 0..3`** at startup
is expected: `IgnitionSystem` implements velocity and position only, which is
why the config sets `use_acceleration_command: false`.

**Killing a run.** `Ctrl-C` usually suffices; when it does not:

```bash
pkill -f "ros2 launch"; pkill -f "ign gazebo"; pkill -f parameter_bridge; ros2 daemon stop
```
