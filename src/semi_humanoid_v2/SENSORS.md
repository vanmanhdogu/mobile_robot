# semi_humanoid_v2 — Sensor and Actuator Layout

Sensor and motor placement for the three robot models in this package:

| Model file | Launch file | Spawned as | 2D LiDAR mounts |
|---|---|---|---|
| `urdf/semi_humanoid_v2.urdf.xacro` | `launch/gazebo.launch.py` | `semi_humanoid_v2` | upright, **not** calibratable |
| `urdf/semi_humanoid_v2_real.urdf.xacro` | `launch/robot_sensor_gazebo.launch.py` | `semi_humanoid_v2` | inverted, calibratable (matches the real ANTBot) |
| `urdf/semi_humanoid_v2_airy.urdf.xacro` | `launch/robot_sensor_airy_gazebo.launch.py` | `semi_humanoid_v2` | as the real model — it `include`s it unchanged |

All three carry `<robot name="semi_humanoid_v2">`. They share the same
chassis, cargo box, OpenArm column and S10 camera ring; they differ only in
the 2D LiDAR mounts, in which 3D LiDARs they carry, and in which extrinsics
the calibration yaml can override. The **airy** model is the real model plus
two RoboSense Airy hemispherical 3D LiDARs — it includes
`semi_humanoid_v2_real.urdf.xacro` verbatim and adds nothing else, so every
real-model note applies to it too. Everything below applies to all three
unless a row or note says otherwise.

Every pose is the sensor frame relative to **`base_link`**, REP-103:
**+X forward, +Y left, +Z up**, `rpy` applied as roll-pitch-yaw.

**No sensor rides the lift or the arms.** Every sensor frame is a fixed child
of `base_link`, so `openarm_lift_joint` does not move any of them and all
poses here are static.

> This robot is **not** the stock ANTBot. The cargo body, GNSS and 3D LiDAR
> are gone; an OpenArm v2.0 bimanual column and a closed cargo box take their
> place. Poses that differ from `antbot_description/urdf/antbot.xacro` are
> listed under [Divergence from the real robot](#divergence-from-the-real-robot).

## Datum — `base_link` is on the ground

> ### ⚠ This package does **not** share `antbot_description`'s datum
>
> `antbot_description` sets `steering_height = 0.120` against
> `wheel_radius = 0.103`, which puts its ground plane at `base_link` z = +0.017
> — i.e. `base_link` sits **17 mm below** the ground.
>
> Here `steering_height = 0.103 = wheel_radius`, so the wheel contact patch is
> at **z = 0** and **`base_link` lies on the ground plane**. Every z in this
> document is therefore height above ground directly; no offset to subtract.
>
> Every z inherited from `antbot_description` is re-datumed by **−0.017 m** in
> this package. That includes the S10 front/back mounts, which can no longer
> use the shared `s10_front_xyz` / `s10_back_xyz` strings and are spelled out
> in `urdf/s10_ring_openarm.xacro` instead — **if the upstream x/y ever move,
> those two lines must be updated by hand.** The pitches are unaffected by the
> datum and still come from upstream.
>
> `base_chassis.stl` is baked in the old datum, so the `base_link` visual
> carries `<origin xyz="0 0 -0.017"/>` rather than being re-exported.

Nothing else in the workspace assumes the old datum: the swerve controller
(`config/controllers_gazebo.yaml`) and `semi_humanoid_navigation`'s nav2 and
SLAM configs reference `base_link` in x–y only.

The Gazebo spawn height is still `-z 0.15` in both launch files. With the new
datum that is a 0.15 m drop onto the wheels instead of 0.133 m — harmless,
but it is now literally "spawn 15 cm above the floor".

## Reference heights

With `base_link` on the ground, **z = height above ground**.

| Datum | z |
|---|---|
| **Ground plane / wheel contact** | **0.000 m** |
| Wheel axle (`steering_height` = `wheel_radius`) | +0.103 m |
| Wheel top | +0.206 m |
| Swerve steering-module top (`steering_link_*.stl`) | +0.242 m |
| `base_chassis.stl` underside / top | +0.040 m / +0.316 m |
| Arm base plate (`openarm_body_link0`) | +0.315 m … +0.379 m |
| Cargo box envelope | +0.316 m … +0.766 m |
| Lift rail (60×60, `x` 0.070…0.130) | +0.379 m … +1.065 m |
| **Arm shoulder centre line**, lift `q = 0` (home) | **+0.815 m** |
| Arm shoulder centre line, lift `q = −0.34` (bottom stop) | +0.475 m |
| **Top crossbar** (travel stop, `x` 0.080…0.120, `y` ±0.373) | **+0.895 m … +0.935 m** |

### Body envelopes the sensor notes refer to

| Part | x | y | z |
|---|---|---|---|
| `base_chassis.stl` (visual, with the −0.017 origin) | −0.3825 … 0.3625 | ±0.2635 | 0.040 … 0.316 |
| `base_link` collision box | −0.383 … 0.363 | ±0.264 | 0.087 … 0.310 |
| Swerve wheels (widest body point) | ±0.265 ±0.103 | ±0.231 … ±0.281 | 0.000 … 0.206 |
| Cargo box, collision | −0.354 … −0.001 | ±0.259 | 0.317 … 0.765 |
| Cargo box, visual incl. frame posts | −0.355 … 0.000 | ±0.260 | 0.316 … 0.766 |
| Arm column base plate | −0.055 … 0.195 | ±0.095 | 0.315 … 0.379 |
| Arm column, full mesh | −0.055 … 0.195 | ±0.095 | 0.315 … 1.065 |
| Top crossbar | 0.080 … 0.120 | ±0.373 | 0.895 … 0.935 |

The cargo box reaches **x = 0.000**, not −0.065: its front wall is notched
around the column base plate, but the notch panels still run from x = −0.059
to −0.001.

## What the launch files actually bring up

The xacro args `camera`, `side_cameras` and `lidar_3d` all default to
**`false`** in both `.urdf.xacro` files; the launch files override them:

| Arg | `gazebo.launch.py` | `robot_sensor_gazebo.launch.py` | `robot_sensor_airy_gazebo.launch.py` |
|---|---|---|---|
| `camera` (front S10) | **true** | false | false |
| `side_cameras` (back/left/right S10) | **true** | false | false |
| `lidar_3d` (stock ANTBot mount) | *not declared* | false | false |
| `airy` (front Airy) | *not declared* | *not declared* | **true** |
| `airy_back` (rear Airy) | *not declared* | *not declared* | **true** |
| `airy_vertical_samples` | — | — | 96 (192 / 96 / 48) |

So a bare `ros2 launch semi_humanoid_v2 gazebo.launch.py` starts all four
RGB-D units, while a bare `ros2 launch semi_humanoid_v2
robot_sensor_gazebo.launch.py` starts none of them. The airy launch file takes
every argument of the real one and adds the three above; **both Airys are on
by default**, and the stock `lidar_3d` stays off, so `lidar_3d:=true` there
gives three 3D LiDARs at once. The 2D LiDARs and the IMU are unconditional in
all three — they have no toggle.

| Sensor | gz topic | ROS topic after bridge | Type | Rate |
|---|---|---|---|---|
| `lidar_2d_front` | `/scan_0` | `/scan_0` | `LaserScan` | 15 Hz |
| `lidar_2d_back` | `/scan_1` | `/scan_1` | `LaserScan` | 15 Hz |
| `imu_sensor` | `/imu` | `/imu/data` | `Imu` | 100 Hz |
| — | `/clock` | `/clock` | `Clock` | — |
| S10 `<pos>` | `/camera_s10_<pos>/image` | `/sensor/camera/stereo_<pos>/image_raw` | `Image` | 15 Hz |
| S10 `<pos>` | `/camera_s10_<pos>/depth_image` | `/sensor/camera/stereo_<pos>/depth/image_raw` | `Image` | 15 Hz |
| S10 `<pos>` | `/camera_s10_<pos>/points` | `/sensor/camera/stereo_<pos>/points` | `PointCloud2` | 15 Hz |
| S10 `<pos>` | `/camera_s10_<pos>/camera_info` | `/sensor/camera/stereo_<pos>/camera_info` | `CameraInfo` | 15 Hz |
| `lidar_3d` † | `/lidar_3d/points` | `/lidar_3d_points` | `PointCloud2` | 10 Hz |
| `airy` ‡ | `/airy/points` | `/airy_points` | `PointCloud2` | 10 Hz |
| `airy_back` ‡ | `/airy_back/points` | `/airy_back_points` | `PointCloud2` | 10 Hz |

`<pos>` ∈ `front` (arg `camera`) and `back` / `left` / `right` (arg
`side_cameras`). The ROS-facing names match what the real S10 drivers
publish, so perception nodes need no sim-specific remapping.
† `lidar_3d` exists in `semi_humanoid_v2_real.urdf.xacro` and, through the
include, in the airy model — not in `semi_humanoid_v2.urdf.xacro`.
‡ The Airys exist **only** in `semi_humanoid_v2_airy.urdf.xacro` /
`robot_sensor_airy_gazebo.launch.py`. Each gz `gpu_lidar` also publishes a
one-ring `LaserScan` on the bare topic; only `/points` is bridged.

`robot_charging_coil_link` is a **frame only** — no gz sensor is attached and
nothing is bridged.

## All sensor frames at a glance

z is height above ground. Every row is a fixed child of `base_link`, which is
itself the ground-level reference — see
[Datum](#datum--base_link-is-on-the-ground).

| Frame | x (m) | y (m) | z (m) | roll | pitch | yaw | gz sensor |
|---|---|---|---|---|---|---|---|
| **`base_link`** | **0** | **0** | **0** | **0°** | **0°** | **0°** | — (reference frame, on the ground) |
| `camera_s10_front_link` | 0.380 | 0 | 0.333 | 0° | 30° | 0° | `rgbd_camera` (opt.) |
| `camera_s10_back_link` | −0.390 | 0 | 0.333 | 0° | 30° | 180° | `rgbd_camera` (opt.) |
| `camera_s10_left_link` | 0.000 | 0.350 | 0.550 | 0° | 40° | 90° | `rgbd_camera` (opt.) |
| `camera_s10_right_link` | 0.000 | −0.350 | 0.550 | 0° | 40° | −90° | `rgbd_camera` (opt.) |
| `imu_link` | −0.100 | 0.00675 | 0.3732 | 0° | 0° | 180° | `imu`, 100 Hz |
| `lidar_2d_front_link` | 0.325 | 0 | 0.238 | 0° | 0° \| **180°** | **180°** \| 0° | `gpu_lidar`, 15 Hz |
| `lidar_2d_back_link` | −0.325 | 0 | 0.238 | 0° \| **180°** | 0° | 0° | `gpu_lidar`, 15 Hz |
| `robot_charging_coil_link` | −0.299119 | 0 | 0.153631 | 0° | 0° | 0° | none |
| `lidar_3d_link` † | 0.225332 | 0 | 0.707904 | 0° | 10° | 0° | `gpu_lidar`, 10 Hz |
| `airy_link` ‡ | 0.200 | 0 | 0.920 | 0° | **+90°** | 0° | `gpu_lidar`, 10 Hz |
| `airy_back_link` ‡ | −0.354 | 0 | 0.770 | 0° | **−90°** | 0° | `gpu_lidar`, 10 Hz |

Where two values are separated by `|`, the first is
`semi_humanoid_v2.urdf.xacro` and the **bold** one is
`semi_humanoid_v2_real.urdf.xacro`.

## Cameras — 4× MRDVS S10 RGB-D

Each unit has two frames: `camera_s10_<pos>_link` (body frame, +X out of the
lens) and `camera_s10_<pos>_optical_link` (optical frame, +Z out of the lens,
fixed `rpy = −90°, 0, −90°` relative to the body frame).

| Unit | Optical axis in `base_link` | Housing AABB (80 × 37 × 25 mm, origin on the front face) |
|---|---|---|
| front | (+0.866, 0, −0.500) | x 0.349…0.389, y ±0.040, z 0.317…0.362 |
| back | (−0.866, 0, −0.500) | x −0.399…−0.359, y ±0.040, z 0.317…0.362 |
| left | (0, +0.766, −0.643) | x ±0.040, y 0.319…0.362, z 0.536…0.580 |
| right | (0, −0.766, −0.643) | x ±0.040, y −0.362…−0.319, z 0.536…0.580 |

**Pitches are 30° / 30° / 40° / 40°.** Only the front pitch comes from
`antbot_description`; **back is overridden from the shared 15° to 30°, and the
sides from the shared 35° to 40°**, in `urdf/s10_ring_openarm.xacro`. Front
and back share a height (0.333 m) and tilt, so their coverage is identical;
the sides sit higher, at 0.550 m, and 10° steeper.

Mounting context in *this* model:

| Unit | Sits on / beside |
|---|---|
| front | link origin 17.5 mm ahead of the chassis mesh front face (0.3625 m) and 17 mm above its top (0.316 m); the tilted housing overhangs back to x = 0.349 but bottoms out at z = 0.317, clearing the chassis top by **1 mm**. 0.185 m ahead of the arm column (column front face x = 0.195 m). |
| back | link origin 35 mm behind the cargo-box rear face (−0.355 m) and 7.5 mm behind the chassis rear face (−0.3825 m); the housing spans x −0.399…−0.359 and bottoms out at z = 0.317, clearing the chassis top (0.316 m) by **1 mm** — the same margin as the front unit. At the shared 15° pitch it bottomed out at 0.315 and grazed the mesh; the 30° pitch removes that. |
| left / right | **beside the cargo box's front corner post** (x −0.020…0.000, \|y\| 0.240…0.260, z 0.316…0.766), at mid-post height: 0.090 m outboard of it and 9 mm above the box's mid-height (0.541 m). Also 0.0865 m outboard of the chassis mesh (±0.2635 m), 0.234 m above the chassis top, and 0.069 m outboard of the widest body point — the swerve wheels at ±0.281 m. 0.315 m below the arm top crossbar. |

> ⚠ **All four positions are estimates**, not CAD. Front and back keep
> `antbot_description`'s x/y (re-datumed in z, see
> [Datum](#datum--base_link-is-on-the-ground)); only their pitches and the two
> side units are overridden here, in `urdf/s10_ring_openarm.xacro`. The shared
> ring would put the sides at (−0.100, ±0.320, 0.633) on the stock ANTBot rack
> post, which this build does not have. They now sit at
> **(0.000, ±0.350, 0.550)** on the cargo box, pitched **30°**.
>
> **The cargo box's front corner post is the mount.** It runs x −0.020…0.000,
> |y| 0.240…0.260, z 0.316…0.766 — each unit sits beside it at mid-post
> height, 90 mm outboard. A ~90 mm bracket off the post reaches it. **The
> bracket is still not modelled**, so the units render as floating and
> self-occlusion checks are optimistic by whatever the real bracket occupies.
>
> This placement clears both conflicts the earlier (0.100, ±0.370, 0.913)
> mount had: the housing is **0.315 m below** `openarm_top_crossbar_link`
> instead of buried in it, and with the lift up (shoulders at z = 0.815)
> `link2` sits 0.169 m above the housing.
>
> ⚠ **Tight in both axes with the lift down.** At the bottom stop the
> shoulders drop to z = 0.475 and the grippers reach the ground. At zero joint
> angles the left `link2` occupies x 0.052…0.135, z 0.409…0.524, so the
> housing clears it by only **12 mm in x and 12 mm in z** — and
> `openarm_*_joint1` (axis Y, range −200°…+80°) can swing the arm back through
> x = 0 on the way down. Keep the cameras in the planning scene rather than
> relying on the nominal clearance.

### Published frame — not the optical frame

`RgbdCameraSensor` sets `ignition_frame_id` to the **mount link**, so image,
depth, point cloud and `camera_info` all carry `frame_id =
camera_s10_<pos>_link` (x-forward), **not** `camera_s10_<pos>_optical_link`.
This is correct for the point cloud — gz-sensors emits it x-forward — but it
is *not* what `image_geometry` and other `camera_info` consumers normally
expect. The optical frames exist in TF; consumers that need them must
transform explicitly.

### Coverage as simulated

The sim frustum is **not** the datasheet frustum. gz derives vfov from hfov
and the image aspect ratio: `hfov = 90°` at 240×160 (3:2) gives
**vfov = 67.38°** (±33.69°), i.e. 7.4° *wider* than the S10's 60°. For an
exact 90×60° frustum pass `height="139"` to the macro.

| Unit | Height | Pitch | Upper edge | Lower edge | Nearest visible ground |
|---|---|---|---|---|---|
| front | 0.333 m | 30° | 3.7° **above** horizontal | 63.7° down | 0.165 m |
| back | 0.333 m | 30° | 3.7° **above** horizontal | 63.7° down | 0.165 m |
| left / right | 0.550 m | 40° | 6.3° **below** horizontal | 73.7° down | 0.161 m |

"Nearest visible ground" is horizontal distance outward from the unit's own
ground projection. Relative to the body: the front unit first sees ground
≈ 0.165 m beyond its mount (x ≈ 0.545 m, 0.18 m past the chassis front face),
the back unit ≈ 0.165 m behind its mount (x ≈ −0.555 m, 0.17 m past the
chassis rear face), and each side unit ≈ 0.161 m outboard of its mount
(|y| ≈ 0.511 m — **0.230 m clear of the wheels**, the widest body point at
|y| = 0.281 m).

**The 0.3 m near clip does not bind anywhere.** The lower-edge ground hit is
at a slant range of 0.372 m (front and back) and 0.573 m (sides), all outside
the clip plane — the sides clear it by 0.273 m, front and back by only 0.072 m
each, which makes them the tightest of the four. On hardware the S10's 0.3 m
minimum range applies the same way.

**Upper edge.** A unit sees nothing above its own upper-edge line. Front and
back, at 30°, look **3.7° above** horizontal and so have no blind ceiling —
each sees progressively higher with distance. The side units, at 40°, look
**6.3° below** horizontal: they never see the horizon, and **nothing at or
above their own 0.550 m mount height is visible to them, at any distance.**

That ceiling is what the side mount buys its near reach and clip margin with.
Across the side mounts tried:

| Side mount | Upper edge | Nearest ground, from robot centre | Near-clip margin | Blind above |
|---|---|---|---|---|
| **0.550 m, 40°** (current) | **6.3° below horizontal** | **0.511 m** | **0.273 m** | **0.550 m** |
| 0.330 m, 30° | 3.7° above | 0.513 m | 0.068 m | — (none) |
| 0.550 m, 30° | 3.7° above | 0.622 m | 0.314 m | — (none) |
| 0.913 m, 41° | 7.3° below | 0.620 m | 0.647 m | 0.913 m |
| 0.633 m, 35° (shared) | 1.3° below | 0.634 m | 0.363 m | 0.633 m |

The 0.330 m / 30° option reached the same distance with no ceiling at all, but
left only 68 mm of near-clip margin. The current mount trades that ceiling for
205 mm more margin.

(At the datasheet's 60° vfov front and back would have an exactly horizontal
upper edge and the sides 10° below — worse again.)

**Azimuth.** Four units with a 90° hfov at yaw 0° / 90° / 180° / −90° tile
the full circle with **zero overlap margin**: each covers exactly ±45° and
the seams abut *in direction*. They are not co-located, though, so each seam
is a blind strip of constant width, set by how far apart the two units sit
across their shared 45° boundary:

| Seam | sides at x = −0.100 | at x = +0.100 | **at x = 0.000** |
|---|---|---|---|
| front ↔ left / right | 0.601 m | 0.445 m | **0.516 m** |
| back ↔ left / right | 0.467 m | 0.594 m | **0.523 m** |

At x = 0.000 the two seams are as near equal as this body allows — neither end
is favoured — but the strips do not close. Any mount error, or the 120×80° variant
(`hfov="${radians(120)}"`), changes this directly.

## IMU

`imu_link` is the only inertial frame, and the only one carrying a gz sensor
(100 Hz, bridged to `/imu/data`). It is yawed 180°, so its **+X points
backward**.

At (−0.100, 0.00675, 0.3732) it falls inside the cargo-box collision envelope
(x −0.354…−0.001, z 0.317…0.765) — plausible for a box-mounted IMU. x/y are
inherited unchanged from `antbot_description`; only z is re-datumed.

> **`magnetometer_link` has been removed from this package.**
> `antbot_description` places it at (0.240, −0.00425, 0.3902): the **same z as
> `imu_link` to 0.1 mm** and within 11 mm in y, but **0.340 m away in x**.
> Those first two agree with a single 9-axis IMU board; the third does not —
> one of the figures is a transcription error, and without CAD there is no way
> to tell which. It also had no gz sensor attached, nothing in this workspace
> subscribed to it, and no `MagneticField` topic was ever bridged, so it was a
> dead frame carrying a self-contradictory pose. Re-add it from
> `antbot_description` once a measured position exists.

## LiDAR — 2× 2D

Both: `gpu_lidar`, 720 samples over a full 360°, range 0.60–20.0 m, 15 Hz,
Gaussian noise σ = 0.008 m. `scan_0` is visualised in the GUI, `scan_1` is
not (720 rays/frame is the dominant rendering cost here).

**Orientation differs between the two models.** `semi_humanoid_v2.urdf.xacro`
mounts both units upright and yaws the front one 180°;
`semi_humanoid_v2_real.urdf.xacro` mounts them inverted, as on hardware
(front: pitch 180°, rear: roll 180°). The positions are identical.

**Both units sit inside the chassis visual mesh.** `base_chassis.stl` spans
x −0.3825…0.3625, y ±0.2635, z 0.040…0.316, so each LiDAR origin is embedded
37.5 mm (front) and 57.5 mm (back) behind its own face, with the scan plane
**78 mm** below the mesh top. The swerve steering modules also reach
z = 0.242, poking 4 mm above the scan plane at x ≈ ±0.265, |y| ≈ 0.17…0.27.
All of that is closer than the 0.60 m minimum range and is therefore
discarded, but the 0.60 m floor means the first possible return is
**0.925 m ahead of `base_link`** (0.925 m behind, for the rear unit) — well
outside the chassis, whose furthest corner is only 0.465 m from `base_link`.
Nothing within that radius is detectable by the 2D scanners at all.

Neither unit is occluded by the payload: the cargo box starts at z = 0.317 m
and the arm base plate at z = 0.315 m, both above the 0.238 m scan plane, and
the wheels top out at z = 0.206 m, below it.

## LiDAR — stock 3D mount (real + airy models, `lidar_3d:=true`)

`semi_humanoid_v2_real.urdf.xacro`, and the airy model through it, can instantiate `Lidar3DSensor` at the
re-datumed stock ANTBot mount, `(0.225332, 0, 0.707904)`, pitch 10°:
`gpu_lidar`, 900 × 32 rays, −25°…+15° vertical, 0.10–70.0 m, 10 Hz, σ = 0.01 m.
`semi_humanoid_v2.urdf.xacro` has no such option; `gazebo_plugins.xacro`
still defines the macro for the real model's use.

**It is heavily occluded by this build.** The lift rail (x 0.070…0.130,
y ±0.030) stands 0.10–0.16 m directly behind it and blocks roughly ±17° of
rearward azimuth. The cargo-box roof at z = 0.766 — 58 mm above the sensor,
starting 0.226 m behind it — cuts everything below ≈ +14° elevation across
≈ ±49° of rearward azimuth, i.e. essentially all rearward ground return. The
chest and arms, which ride the lift between z = 0.475 and 0.815, sweep through
the rest of the rearward field. It is provided so that `antbot_gazebo`'s
`lidar_3d` option behaves the same way here, not because this robot has one.

`lidar_3d_imu_link`, which `antbot.xacro` adds relative to `lidar_3d_link`,
does **not** exist in either model here.

## LiDAR — 2× RoboSense Airy (airy model only)

`semi_humanoid_v2_airy.urdf.xacro` adds two hemispherical 3D LiDARs on top of
the real model. Both are on by default.

| Frame | x | y | z | rpy | Dome axis | Hemisphere covers |
|---|---|---|---|---|---|---|
| `airy_link` (front) | 0.200 | 0 | 0.920 | 0°, **+90°**, 0° | **+X** | everything at x > 0.200 |
| `airy_back_link` (rear) | −0.354 | 0 | 0.770 | 0°, **−90°**, 0° | **−X** | everything at x < −0.354 |

The link origin is at the centre of the **mounting face**, with +Z along the
dome axis; the body occupies 0…0.063 m along that axis, Ø60 mm. The real
optical centre is not published.

| Parameter | Value | From |
|---|---|---|
| Horizontal | 900 samples over 360° = **0.4°** | datasheet |
| Vertical | `airy_vertical_samples` over **0°…+90°** | 192 / 96 / **48**; default 96 → 0.94°/line |
| Range | 0.10 … 60.0 m | datasheet blind spot / max |
| Rate | 10 Hz | datasheet |
| Noise | σ = 0.01 m | `Lidar3DSensor` default, matches the ±1 cm spec |
| Housing | Ø60 × H63 mm, 240 g | datasheet, `meshes/airy_lidar.stl` |

**Vertical 0…+90° is elevation from the sensor's own mounting plane**, so each
unit covers the hemisphere on its +Z side. After the ±90° pitch, that is the
half-space ahead of (front) or behind (rear) its own mounting plane.

### ⛔ The two hemispheres do not cover the sphere — there is a 0.554 m blind slab

Both the xacro and the launch file say the pair "see the whole sphere around
the robot". That is true of two **co-located** hemispheres. These two sit
**0.554 m apart in x**, and a hemisphere's boundary is a hard plane through its
own origin, so:

- the front unit sees only **x > 0.200**,
- the rear unit sees only **x < −0.354**,
- **nothing sees −0.354 < x < 0.200** — a vertical slab through the middle of
  the robot, unbounded in y and z.

A point abeam at (0, 2.0, 1.0) is behind the front unit's plane (Δx = −0.200)
and ahead of the rear unit's plane (Δx = +0.354): invisible to both. The slab
swallows everything directly to the robot's left and right across the whole
mid-body, which is exactly where the side S10s have their own 0.550 m ceiling.
**Neither 3D nor RGB-D sees a tall object abeam of the mid-body.**

Fixes, in order of how little they disturb: move the two mounts toward each
other in x until the slab closes (they must share an x plane), tilt each unit
so its boundary plane rakes past the other's, or accept the slab and treat the
Airys as forward/rear sensors rather than a spherical pair.

### Mounting context

| Unit | Body in `base_link` | Sits on |
|---|---|---|
| front | x 0.200…0.263, y ±0.030, z 0.890…0.950 | **nothing in the model.** At z = 0.92 the arm column is only the 60×60 rail (x 0.070…0.130), so the mounting face floats **70 mm** ahead of it. The nearest real structure is the **top crossbar** (x 0.080…0.120, z 0.895…0.935, full width through y = 0) — an 80 mm bracket off its front face reaches the mount. |
| rear | x −0.417…−0.354, y ±0.030, z 0.740…0.800 | flush on the **cargo box rear panel** (rear face x = −0.354), at its top edge — the box roof is at 0.766 and the unit's axis at 0.770. It overlaps the box's rear top rail (x −0.355…−0.335, z 0.746…0.766) by **≈1 mm**. |

The xacro comment describes both as "back to back on the body column"; neither
is. The column rail ends 70 mm short of the front unit, and the rear unit is
0.48 m behind the rail, on the cargo box.

### Self-hits

`range_min` is 0.10 m, so each unit's own dome (0.063 m) is inside the blind
spot and does not occlude. The robot is not:

- the **front** unit sees everything at x > 0.200 — the chassis top out to
  x = 0.3625, and the front S10 housing (x 0.349…0.389, ≈0.61 m away);
- the **rear** unit sees everything at x < −0.354 — the last 28 mm of the
  chassis and the back S10 housing (x −0.399…−0.359);
- the arms, whose shoulders sit at x = 0.100 (inside the slab), enter the
  front hemisphere whenever `openarm_*_joint1` swings them past x = 0.200.

The top crossbar, the cargo box and the side S10s are all inside the blind
slab and so never appear in either cloud.

### Render cost

Rays per frame are `horizontal_samples × vertical_samples`, both units at once:

| `airy_vertical_samples` | Per unit | Both units | vs the stock `lidar_3d` (900 × 32) |
|---|---|---|---|
| 192 | 172 800 | 345 600 | 12× |
| **96** (default) | 86 400 | 172 800 | **6×** |
| 48 | 43 200 | 86 400 | 3× |

At 10 Hz the default pair is 1.73 M rays/s. Drop to 48 lines before dropping
anything else if the sim falls behind.

## Other

| Frame | x (m) | y (m) | z (m) | rpy |
|---|---|---|---|---|
| `robot_charging_coil_link` | −0.299119 | 0 | 0.153631 | 0°, 0°, 0° |

## Motors — 29 actuated joints

Every joint below is in `urdf/ros2_control_gazebo.xacro` under one
`ign_ros2_control/IgnitionSystem`; Gazebo loads a single controller manager
per model, so the base, the lift and both arms share it. Positions are the
**joint axis origin in `base_link`** with the robot at its home pose
(lift `q = 0`, all arm joints 0); limits are the **real** ones — in sim every
arm stop is widened by 0.02 rad and the lift by 0.005 m (see
`openarm_sim_limit_margin`), so a command inside the real range never touches
a stop.

### Swerve base — 8 motors

Geometry from `wheelbase_length` 0.530, `steering_width` 0.401,
`wheel_offset` (0.512 − 0.401)/2 = 0.0555, `steering_height` = `wheel_radius`
= 0.103. The steering axis and the wheel axle are **55.5 mm apart in y** — the
modules are offset, not centred.

| Joint | x | y | z | Axis | Range | Effort / velocity |
|---|---|---|---|---|---|---|
| `steering_front_left_joint` | 0.265 | 0.2005 | 0.103 | Z | −90°…+90° | 1000 / 6.5 |
| `steering_front_right_joint` | 0.265 | −0.2005 | 0.103 | Z | −90°…+90° | 1000 / 6.5 |
| `steering_rear_left_joint` | −0.265 | 0.2005 | 0.103 | Z | −90°…+90° | 1000 / 6.5 |
| `steering_rear_right_joint` | −0.265 | −0.2005 | 0.103 | Z | −90°…+90° | 1000 / 6.5 |
| `wheel_front_left_joint` | 0.265 | 0.2560 | 0.103 | Y | continuous | 200 / 50 |
| `wheel_front_right_joint` | 0.265 | −0.2560 | 0.103 | Y | continuous | 200 / 50 |
| `wheel_rear_left_joint` | −0.265 | 0.2560 | 0.103 | Y | continuous | 200 / 50 |
| `wheel_rear_right_joint` | −0.265 | −0.2560 | 0.103 | Y | continuous | 200 / 50 |

Steering takes a **position** command, the wheels a **velocity** command.
`config/controllers_gazebo.yaml` repeats the same geometry for
`antbot_swerve_controller` as `module_x_offsets` ±0.265,
`module_y_offsets` ±0.256, `steering_to_wheel_y_offsets` ±0.0555 and
`wheel_radius` 0.103 — **these four must be kept in step with the URDF by
hand**; nothing cross-checks them.

### Lift — 1 motor

| Joint | x | y | z | Axis | Range | Effort / velocity |
|---|---|---|---|---|---|---|
| `openarm_lift_joint` | 0.100 | 0 | 0.815 | Z (prismatic) | −0.340 … 0 m | 2000 / 0.10 |

`q = 0` is the **top** stop (home); `q = −0.340` is the bottom, where the
grippers reach the ground. The position given is the carriage at `q = 0`,
which is also the shoulder centre line. Damping 50.0.

### Arms — 14 motors (7 per arm)

Both chains hang off `openarm_lift_link`, so **every z below shifts with the
lift** — subtract up to 0.340 m. Joints 3…7 are collinear in x–y with joint 2;
only z changes.

| Joint (left / right) | x | y (left) | y (right) | z | Axis, left | Axis, right | Range, left | Range, right | Eff / vel |
|---|---|---|---|---|---|---|---|---|---|
| `…_joint1` | 0.100 | 0.2485 | −0.2485 | 0.8150 | +Y | −Y | −200°…+80° | −80°…+200° | 40 / 16.755 |
| `…_joint2` | 0.100 | 0.3085 | −0.3085 | 0.8150 | −X | −X | −190°…+10° | −10°…+190° | 40 / 16.755 |
| `…_joint3` | 0.100 | 0.3085 | −0.3085 | 0.7487 | −Z | −Z | −90°…+90° | −90°…+90° | 27 / 5.4454 |
| `…_joint4` | 0.100 | 0.3085 | −0.3085 | 0.5950 | −Y | −Y | 0°…+140° | 0°…+140° | 27 / 5.4454 |
| `…_joint5` | 0.100 | 0.3085 | −0.3085 | 0.4995 | −Z | −Z | −90°…+90° | −90°…+90° | 7 / 20.944 |
| `…_joint6` | 0.100 | 0.3085 | −0.3085 | 0.3790 | −Y | **+Y** | −45°…+45° | −45°…+45° | 7 / 20.944 |
| `…_joint7` | 0.100 | 0.3085 | −0.3085 | 0.3790 | +X | +X | −90°…+90° | −90°…+90° | 7 / 20.944 |

Joints 6 and 7 share an origin — a wrist intersection. The two arms are
**mirrored, not copied**: joint1 and joint6 flip axis sign and joints 1–2 flip
their limit ranges, so a joint command is not interchangeable between arms.

> ⚠ `joint4`'s lower limit is **exactly 0**, and the arms spawn at all-zero —
> i.e. on the stop. That is why `openarm_sim_limit_margin` exists: with
> `gz_ros2_control` applying position commands as DART velocity commands, a
> joint sitting on its limit stops responding (gazebosim/gz-sim#1684).

### Grippers — 4 motors (2 per gripper, 1 mimicked)

| Joint | x | y (left) | y (right) | z | Axis | Range, left | Range, right | Eff / vel |
|---|---|---|---|---|---|---|---|---|
| `…_finger_joint1` | 0.0986 | 0.2905 | −0.2905 | 0.3110 | −X | 0°…+45° | −45°…0° | 7 / 5 |
| `…_finger_joint2` | 0.0986 | 0.3265 | −0.3265 | 0.3110 | +X | 0°…+45° | −45°…0° | 7 / 5 |

`finger_joint2` carries `<param name="mimic">…finger_joint1</param>` with
multiplier 1, so each gripper is one degree of freedom driven through
`finger_joint1`. Like the arms, the left and right ranges are mirrored. These
are the only joints whose origin is not at x = 0.100 — their mounting frames
carry a non-zero rpy.

### Controllers

`config/controllers_gazebo.yaml`, spawned in this order by both sensor launch
files: `joint_state_broadcaster` → `antbot_swerve_controller` →
`lift_controller`, `left_arm_controller`, `right_arm_controller`,
`left_gripper_controller`, `right_gripper_controller`.

## Calibration override

All three launch files forward `calibration_yaml_path:=<file>` to xacro.
Which frames honour it **differs between the models** — the airy model behaves
exactly like the real one, and the two Airy mounts have no calibration key at
all (move them with the `airy_xyz` / `airy_rpy` / `airy_back_xyz` /
`airy_back_rpy` xacro args instead):

| Key | Frame | `semi_humanoid_v2` | `semi_humanoid_v2_real` |
|---|---|---|---|
| `camera_s10_front_extrinsic` | `camera_s10_front_link` | ✅ | ✅ |
| `camera_s10_back_extrinsic` | `camera_s10_back_link` | ✅ | ✅ |
| `camera_s10_left_extrinsic` | `camera_s10_left_link` | ✅ | ✅ |
| `camera_s10_right_extrinsic` | `camera_s10_right_link` | ✅ | ✅ |
| `lidar_2d_front_extrinsic` | `lidar_2d_front_link` | ❌ ignored | ✅ |
| `lidar_2d_back_extrinsic` | `lidar_2d_back_link` | ❌ ignored | ✅ |
| `lidar_3d_extrinsic` | `lidar_3d_link` | — (no frame) | ✅ (with `lidar_3d:=true`) |

Keys hold `tx/ty/tz` in metres and `rx/ry/rz` in radians, and replace the
whole default origin when present.

> ⛔ **A calibration yaml written for the real robot is now wrong in z by
> 17 mm here.** The override replaces the whole origin, so an upstream
> `tz` lands unshifted in this package's ground-referenced frame. Either
> re-datum the yaml or keep a separate one for this model.

> ⚠ **The 2D LiDARs are not calibratable in the sim model.**
> `semi_humanoid_v2.urdf.xacro` mounts them with the plain `Sensors` macro, so
> `lidar_2d_front_extrinsic` / `lidar_2d_back_extrinsic` in a shared
> calibration yaml are **silently ignored**. The same yaml therefore produces
> different LiDAR extrinsics under `gazebo.launch.py` than on the real robot
> or under `robot_sensor_gazebo.launch.py`.

`imu`, `robot_charging_coil` and both Airys have no override
key in any model — their poses are hard-coded or set by xacro args.

## Divergence from the real robot

Against `antbot_description/urdf/antbot.xacro`. **Every real-robot z below is
in that package's datum (`base_link` 17 mm under the ground); subtract 0.017
to compare with this package's columns.**

| Item | Real robot | `semi_humanoid_v2` | `semi_humanoid_v2_real` |
|---|---|---|---|
| `steering_height` | 0.120 | **0.103** (datum change) | **0.103** |
| All inherited z | — | **−0.017 m** | **−0.017 m** |
| S10 front / back x, y | shared `S10CameraRing` macro | identical | identical |
| S10 front / back z | 0.350 | **0.333** (datum only) | **0.333** |
| S10 front pitch | 30° | 30° (upstream) | 30° (upstream) |
| S10 back pitch | 15° | **30°** | **30°** |
| S10 left / right x | −0.100 | **0.000** (cargo-box front edge) | **0.000** |
| S10 left / right y | ±0.320 | **±0.350** (30 mm wider) | **±0.350** |
| S10 left / right z | 0.650 (rack post) | **0.550** (−0.083 after datum) | **0.550** |
| S10 left / right pitch | 35° | **40°** | **40°** |
| `imu_link` x, y | as tabled above | identical | identical |
| `magnetometer_link` | present at (0.240, −0.00425, 0.350) | **removed** | **removed** |
| `robot_charging_coil_link` x, y | as tabled above | identical | identical |
| `lidar_2d_front_link` rpy | `0°, 180°, 0°` | **`0°, 0°, 180°`** | identical |
| `lidar_2d_back_link` rpy | `180°, 0°, 0°` | **`0°, 0°, 0°`** | identical |
| 2D LiDAR calibration keys | honoured | **ignored** | honoured |
| `gnss_link` (0.2026, −0.128, 0.660) | present | **absent** | **absent** |
| `lidar_3d_link` | present, always | **absent** | optional (`lidar_3d`) |
| `lidar_3d_imu_link` | present | **absent** | **absent** |
| `airy_link`, `airy_back_link` | **absent** | **absent** | airy model only |

The 2D LiDAR x/y match everywhere; only `semi_humanoid_v2.urdf.xacro`'s
orientations differ, and by a real rotation rather than an equivalent
re-parameterisation. The real robot mounts both units inverted (front by a
180° pitch, rear by a 180° roll); the sim model keeps both upright and yaws
the front one, which mirrors the scan direction relative to hardware. That
model follows `antbot_gazebo/urdf/antbot_sim.xacro`, so the mismatch is
inherited, not introduced here — but **treat 2D scan orientation under
`gazebo.launch.py` as unverified against hardware.** Use
`robot_sensor_gazebo.launch.py` when the orientation matters.

## Open items

- **Side cameras vs the arms, lift down** — at (0.000, ±0.350, 0.550) the
  units are clear of the arms whenever the lift is up, but at the bottom stop
  `link2` passes within 12 mm in x and `openarm_*_joint1` can swing the arm
  through x = 0 at that height. Add the cameras to the planning scene; a
  dedicated collision pair or a joint-1 limit may still be wanted.
- **⛔ Calibration yamls are datum-specific** — a yaml measured on the real
  robot is 17 mm off in z when loaded here, silently. Tag the yaml with its
  datum, or convert on load.
- **Datum drift risk** — `urdf/s10_ring_openarm.xacro` now hard-codes the S10
  front/back x/y instead of reading `s10_front_xyz` / `s10_back_xyz`, because
  the shared strings carry the old datum. If upstream moves those mounts in
  x or y, this package will not follow. A shared `s10_*_x` / `_y` / `_z`
  property triple upstream would fix it properly.
- **Side-camera brackets are not modelled** — nothing joins the units to the
  crossbar, so the model shows them floating and self-occlusion checks are
  optimistic by whatever the real bracket occupies.
- **Mount coordinates** — all four S10 positions and pitches are estimates.
  The vendor STEP file and the depth/RGB optical-centre offsets are still
  outstanding, so the link origin sits on the front face of the housing rather
  than at the optical centre.
- **Pitch overrides vs `antbot_description`** — back 30° against the shared
  15°, sides 40° against the shared 35°, plus the side x-y-z move. If the real
  robot keeps 15° / 35°, sim and hardware coverage differ by 15° at the back
  and 5° on each side. The back change halves the rear blind ring
  (0.293 → 0.165 m) but gives up most of the rear upward view; confirm which
  the rear unit is actually for.
- **`base_chassis.stl` is baked in the old datum** — the −0.017 origin on the
  `base_link` visual works, but re-exporting the mesh would remove the one
  place where a reader has to remember the offset.
- **Two models to keep in sync** — `semi_humanoid_v2.urdf.xacro` and
  `semi_humanoid_v2_real.urdf.xacro` duplicate the whole base, wheel and
  payload block and differ only in the LiDAR mounts. Any pose edit has to be
  made twice; factor the shared part into one include.
- **2D LiDAR orientation and calibration** — both diverge from the real robot
  in the sim model only; see the table above.
- **Sim vfov ≠ datasheet** — 67.4° against 60°; decide whether to pass
  `height="139"` for an exact 90×60° frustum before tuning perception on
  simulated coverage.
- **3D LiDAR mount is unusable as placed** — if the real model's `lidar_3d`
  option is ever meant for real use, re-place it above the top crossbar
  (z > 0.935) or drop it.
- **⛔ The Airy pair leaves a 0.554 m blind slab** — front sees x > 0.200,
  rear sees x < −0.354, nothing sees between. The xacro and launch docstrings
  both claim full spherical coverage; they are wrong for non-co-located units.
  Either bring the two mounts onto a common x plane or restate the pair as
  forward/rear sensors. See
  [LiDAR — 2× RoboSense Airy](#lidar--2-robosense-airy-airy-model-only).
- **Front Airy has no structure under it** — its mounting face is at x = 0.200
  while the column rail ends at x = 0.130. Model an 80 mm bracket off the top
  crossbar's front face, or move the unit back to x = 0.130.
- **Rear Airy grazes the cargo box rear rail** — ≈1 mm interpenetration at
  x −0.355…−0.354, z 0.746…0.766. Cosmetic, same class as the back S10.
- **Airy xacro comment says "on the body column"** — neither unit is; the
  front floats ahead of the rail, the rear is on the cargo box. Fix the
  comment when the brackets are modelled.
- **⚠ Side cameras are blind above 0.550 m** — the 40° tilt puts their upper
  edge 6.3° *below* horizontal, so nothing at or above their own mount height
  is ever visible to them: a person, a shelf or a tall load beside the robot
  is simply absent from the side clouds. Front and back (30°) have no such
  ceiling. If the sides are meant for obstacle height and not just near-ground
  coverage, this is the figure to revisit — see the table under
  [Coverage as simulated](#coverage-as-simulated) for what each alternative
  costs.
- **Front and back near-clip margin is only 72 mm** — their lower-edge ground
  hit is at a 0.372 m slant against the 0.3 m clip plane, tighter than the
  sides' 0.273 m. They are the pair to watch if any mount drops further.
- **Cargo camera** — `antbot_camera` publishes `cargo_camera_optical_frame`,
  which has no frame in either model here.
