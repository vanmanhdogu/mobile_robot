# semi_humanoid_navigation

Nav2 navigation for the ANTBot + OpenArm semi-humanoid
(`antbot_openarm_description`). Based on `antbot/antbot_navigation`: same
launch files, configs, depot map and helper scripts, with the robot-specific
parts changed (see [Changes vs. antbot_navigation](#changes-vs-antbot_navigation)).

## Quick start (simulation)

Inside the Docker container (`cd ~/ros2_humble_ws/docker && ./run.sh shell`):

```bash
cd ~/ros2_humble_ws
colcon build --symlink-install --packages-select semi_humanoid_navigation
source install/setup.bash

ros2 launch semi_humanoid_navigation bringup.launch.py
```

That starts Gazebo with the robot in the depot world, AMCL on the saved depot
map, Nav2 and RViz. Navigation starts 15 s after the robot (`nav_delay`), so
wait for it. Then in RViz:

1. **2D Pose Estimate** if the robot in RViz doesn't line up with the map. It
   spawns at the map origin, which is also AMCL's initial pose, so usually
   this isn't needed.
2. **Nav2 Goal**: click and drag a target.

Or send a goal from a shell:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 3.0}, orientation: {w: 1.0}}}}"
```

## bringup.launch.py

One command for robot + navigation (or SLAM) + RViz.

| Argument | Default | Effect |
|---|---|---|
| `mode` | `sim` | `sim`: `antbot_openarm_description/gazebo.launch.py`. `real`: `real_robot.launch.py` plus the 2D LiDAR driver (`antbot_bringup/lidar_2d.launch.py`) |
| `slam` | `false` | `true`: build a map with slam_toolbox instead of navigating |
| `world` | `depot` | Gazebo world (sim), and the map looked up in `maps/worlds.yaml` |
| `map` | empty | map YAML to navigate on; overrides the `world` lookup |
| `rviz` | `true` | RViz with `rviz/navigation.rviz` |
| `headless` | `false` | sim: Gazebo without its window |
| `use_mock_hardware` | `false` | real: mock the base (no ANT-RCU board) |
| `nav_delay` | `15.0` | seconds to wait for the robot before starting navigation or SLAM |

Examples:

```bash
# simulation, navigate on the depot map
ros2 launch semi_humanoid_navigation bringup.launch.py

# simulation, build a new map (drive with /cmd_vel or teleop), then save it
ros2 launch semi_humanoid_navigation bringup.launch.py slam:=true
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -p use_sim_time:=true

# navigate on a saved map
ros2 launch semi_humanoid_navigation bringup.launch.py map:=$HOME/maps/my_map.yaml

# real robot (board on /dev/ttyUSB0, LiDARs on ttyUSB2/3 mapped into Docker)
ros2 launch semi_humanoid_navigation bringup.launch.py mode:=real map:=$HOME/maps/my_map.yaml
```

The lower-level launch files work as in `antbot_navigation` if you start the
robot yourself:

```bash
ros2 launch antbot_openarm_description gazebo.launch.py world:=depot           # terminal 1
ros2 launch semi_humanoid_navigation navigation.launch.py mode:=sim world:=depot # terminal 2
ros2 launch semi_humanoid_navigation slam.launch.py mode:=sim                   # instead of navigation
ros2 launch semi_humanoid_navigation localization.launch.py mode:=sim map:=...   # AMCL only
```

## Modes

| | `mode:=sim` | `mode:=real` |
|---|---|---|
| Robot | `gazebo.launch.py` | `real_robot.launch.py` + 2D LiDAR driver |
| Config | `config/sim/` | `config/real/` |
| Controller | MPPI | Regulated Pure Pursuit |
| LiDAR | `/scan_0` + `/scan_1` | `/scan_0` → `scan_fix_relay` → `/scan_0_fixed` |
| Odometry | swerve controller `/odom` | swerve controller `/odom` |
| `use_sim_time` | true | false |

The architecture, tuning tables and troubleshooting in
`antbot/antbot_navigation/README.md` apply unchanged: the controllers, AMCL,
costmap layers and speeds are the same.

## Changes vs. antbot_navigation

| What | antbot_navigation | here | Why |
|---|---|---|---|
| Footprint (both modes, both costmaps) | 0.70 × 0.60 m rectangle | chassis 0.79 × 0.60 m plus arm "wings" to y = ±0.43 m at x 0.03…0.17 | The OpenArm arms hang beside the chassis out to y = ±0.411 m, below the LiDAR plane. Measured from the URDF collision geometry, +2 cm margin. |
| Sim `bt_navigator.odom_topic` | `/odometry/filtered` | `/odom` | No EKF is launched, so `/odometry/filtered` never exists. |
| `bringup.launch.py` | – | new | Starts the semi-humanoid, navigation or SLAM, and RViz together. |
| `maps/worlds.yaml` | world `.sdf` + map | map only | Gazebo worlds are resolved by `antbot_gazebo`; the depot `.sdf` isn't duplicated. |

### The footprint assumes the arms are down

The footprint matches the arms in their default hanging pose. If the arms are
raised sideways the robot is wider than Nav2 thinks, so keep them in before
navigating, or widen `footprint` in `config/*/nav2_params.yaml`.

On the real robot the 2D LiDARs can also see the arms (the simulated LiDARs
can't: their 0.6 m minimum range hides them). Because the arms are inside the
footprint, the costmap's footprint clearing removes those hits, as long as
the arms stay inside it.

## Tested (simulation)

- `bringup.launch.py`: all 6 robot controllers and both Nav2 lifecycle
  managers active; a goal to (5.0, 3.0) on the depot map **SUCCEEDED** in 14 s.
  AMCL pose (4.89, 2.93) vs. Gazebo ground truth (4.77, 3.03).
- `slam:=true`: the map grows while driving, `map → odom` is published, and
  `map_saver_cli` saves it.
- `mode:=real use_mock_hardware:=true`: robot, LiDAR driver, `scan_fix_relay`
  and the RPP Nav2 stack start. Not tested with real LiDARs or the real board.

## Package structure

```text
semi_humanoid_navigation/
├── config/{sim,real}/   nav2_params.yaml, slam_toolbox_params.yaml, ekf.yaml (ekf unused, as upstream)
├── launch/              bringup.launch.py, navigation.launch.py, localization.launch.py, slam.launch.py
├── maps/                depot_sim.pgm, depot_sim.yaml, worlds.yaml
├── rviz/                navigation.rviz
└── scripts/             scan_fix_relay.py, odom_slam_test.py, odom_slam_plot.py
```
