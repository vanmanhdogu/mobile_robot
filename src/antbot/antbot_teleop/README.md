# antbot_teleop

Teleoperation package for the ANTBot swerve-drive robot. Provides keyboard, DualSense joystick and Logitech F710 joystick control nodes. All nodes publish `geometry_msgs/msg/Twist` on `/cmd_vel`.

## Nodes

### teleop_keyboard

Terminal-based keyboard control. The robot moves only while a key is held and stops on release.

```
   q    w    e
   a         d
        x

w/x       : forward / backward     (linear.x)
a/d       : strafe left / right    (linear.y)
q/e       : rotate CCW / CW        (angular.z)
1~9       : speed level (1=slow, 9=max)
ESC/Ctrl+C: quit
```

> **Note:** Requires a terminal (TTY) for keyboard input. Always run directly in a terminal, not via a launch file.

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_vel` | `1.0` | Maximum linear velocity [m/s] |
| `max_angular_vel` | `1.0` | Maximum angular velocity [rad/s] |
| `speed_level` | `3` | Initial speed level (1~9) |
| `publish_rate` | `10.0` | Publish rate [Hz] |

### teleop_joystick

Joystick control with automatic controller detection (DualSense) and geometry-based angular velocity limiting. Prevents steering angles from exceeding hardware limits by computing maximum angular velocity from robot geometry.

**Supported controllers:** PS5 DualSense (tested), PS4 DualShock 4 (untested) via USB connection. The node auto-detects the controller type via USB product ID, with axis-based fallback detection. Hot-plug is supported — swapping controllers mid-session is automatically handled.

```
[Left Stick]                        [Triggers]
  Y-axis : forward / backward        L2 : in-place rotate CCW
  X-axis : curve turning              R2 : in-place rotate CW
           (while moving)

[Buttons]
  Triangle : speed level UP (+1)     L1 : headlight toggle (ON/OFF)
  Cross    : speed level DOWN (-1)   R1 : wiper toggle (REPEAT/OFF)
  Square   : cargo lock              Circle : cargo unlock
```

**Two driving modes** (mutually exclusive):
- **Curve driving** (`|vx| >= 0.05 m/s`): Left stick X controls angular velocity, constrained by `w_max = min(|vx| / R_min, W_ABS_MAX)`. Steering inverts automatically when reversing.
- **In-place rotation** (`|vx| < 0.05 m/s`): L2/R2 triggers control spin. `wz = max_spin_vel * speed_ratio * (L2 - R2)`.

**Service calls:** Square/Circle buttons call `cargo/command` (`CargoCommand`). L1 toggles `headlight/operation` (`SetBool`). R1 toggles `wiper/operation` (`WiperOperation`).

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_vel` | `1.0` | Maximum linear velocity [m/s] |
| `max_spin_vel` | `1.0` | Maximum in-place rotation speed [rad/s] |
| `speed_level` | `3` | Initial speed level (1~9) |
| `deadzone` | `0.1` | Stick/trigger deadzone threshold |
| `module_x` | `0.265` | Swerve module X-offset from base center [m] |
| `module_y` | `0.256` | Swerve module Y-offset from base center [m] |
| `steering_limit_deg` | `60.0` | Hardware steering limit [deg] |
| `safety_factor` | `0.95` | Use 95% of steering limit (3 deg margin) |
| `w_abs_max` | `2.0` | Absolute max angular velocity [rad/s] |

### teleop_joystick_f710

Holonomic joystick control for the **Logitech F710** wireless gamepad, implementing the control table in `joystick.md`. Unlike `teleop_joystick` (Sony pads, curve-driving model) this node drives all three `Twist` channels — `linear.x`, `linear.y` and `angular.z` — and requires a dead-man switch.

> **The rear D/X switch must be on X (XInput).** In X mode the pad enumerates 8 axes and 11 buttons; in D (DirectInput) mode it enumerates differently and the node refuses to drive, logging the axis/button counts it saw.

```
RB (hold)     : dead-man switch -- the robot only moves while RB is held
Left stick Y  : forward / backward          (linear.x, analog)
Right stick X : rotate CCW / CW in place    (angular.z, analog)
D-pad up/down : forward / backward step     (linear.x, x dpad_gain)
D-pad left/rt : strafe left / right         (linear.y, x dpad_gain)
LB (hold)     : force speed level 1 (slowest)
LT (hold)     : force speed level 9 (fastest)
Y / A         : speed level +1 / -1 (rising edge only)
X / B         : cargo lock / unlock
BACK / START  : headlight / wiper toggle
```

**Speed model.** Level `L` in 1..9 scales both limits linearly: `v = max_linear_vel * L/9`, `w = max_angular_vel * L/9`. Priority is **LB > LT > base level**, so grabbing LB always wins and both are momentary — releasing returns to the level set with Y/A.

```
linear.x  = clamp(dz(stick_y) * v + dpad_y * v * dpad_gain, +/-v)
linear.y  = clamp(              dpad_x * v * dpad_gain,     +/-v)
angular.z = clamp(dz(rstick_x) * w,                         +/-w)
```

`dz()` is a rescaling deadzone: zero below `deadzone`, and the remainder stretched back out to the full +/-1.0 range so there is no step at the edge.

#### Safety behaviour

| Hazard | Handling |
|---|---|
| Unintended motion | RB dead-man switch. Releasing RB publishes a zero `Twist` immediately; nothing is published at all while RB is up. |
| 2.4 GHz link loss | Watchdog: no `/joy` for `watchdog_timeout` publishes a bounded burst of zero `Twist` and logs a warning. Pair with `autorepeat_rate: 20.0` on `joy_node`. |
| Trigger reads `0.0` before its first event | `LT held` is `axes[lt] < lt_threshold` with `lt_threshold = -0.5`, so an uninitialised `0.0` reads as *released*. A `< +0.5` test would jump to level 9 at power-on. |
| Speed level running away | Y and A latch on the rising edge only, so holding Y steps the level once. |
| MODE button swapping stick and D-pad | The node cross-checks live axis values: a fractional reading on an axis it believes is the D-pad is impossible for a hat, so it swaps the profile and logs an error. If the stick only ever reports +/-1.0 it warns that MODE may be on. |
| Held button firing on re-engage | Button edge state is refreshed while the dead-man is released, so a button held across the release does not fire when RB is pressed again. |

#### Axis profile

`joystick.md` section 2.1 records the left stick on `axes[6..7]` and the D-pad on `axes[0..1]`. On this machine the pad reports the opposite, which is what `joy_node` produces for an F710 in XInput mode: the stick is SDL axes 0/1 and the D-pad hat is appended as axes 6/7. The `axis_profile` parameter selects between them, and the runtime cross-check above corrects a wrong choice on its own.

| `axis_profile` | Left stick | D-pad |
|---|---|---|
| `standard` (default) | `axes[0]`, `axes[1]` | `axes[6]`, `axes[7]` |
| `mode_swapped` | `axes[6]`, `axes[7]` | `axes[0]`, `axes[1]` |

Confirm which one applies with `ros2 topic echo /joy --field axes` while pushing the left stick halfway: the pair showing fractional values is the stick.

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_vel` | `1.0` | Linear velocity at speed level 9 [m/s] |
| `max_angular_vel` | `1.0` | Angular velocity at speed level 9 [rad/s] |
| `speed_level` | `3` | Initial speed level (1~9) |
| `deadzone` | `0.12` | Stick deadzone, rescaled |
| `dpad_gain` | `0.5` | D-pad step as a fraction of full speed |
| `lt_threshold` | `-0.5` | LT value below which it counts as held |
| `watchdog_timeout` | `0.3` | Seconds without `/joy` before stopping [s] |
| `zero_repeat` | `5` | Zero `Twist` messages sent when motion stops |
| `axis_profile` | `standard` | `standard` or `mode_swapped` (see above) |
| `enable_accessories` | `true` | Enable cargo / headlight / wiper service calls |
| `axis_lt` / `axis_rt` / `axis_rstick_x` | `2` / `5` / `3` | Fixed axis indices |
| `btn_deadman` … `btn_wiper` | `5` / `4` / `3` / `0` / `2` / `1` / `6` / `7` | Button indices (XInput order) |

Accessory service calls are optional: if `antbot_interfaces` is missing or built for a different ROS distro, the node logs an error, disables those four buttons and keeps driving.

## Usage

```bash
# Keyboard teleop (run in terminal)
ros2 run antbot_teleop teleop_keyboard

# Joystick teleop (DualSense via USB)
ros2 launch antbot_teleop teleop_joy.launch.py

# Joystick teleop (Logitech F710, rear switch on X)
ros2 launch antbot_teleop teleop_joy_f710.launch.py

# ...with your own parameter file
ros2 launch antbot_teleop teleop_joy_f710.launch.py \
  params_file:=/path/to/joy_f710.yaml
```

Override parameters via command line:
```bash
ros2 run antbot_teleop teleop_keyboard --ros-args -p max_linear_vel:=0.5
ros2 run antbot_teleop teleop_joystick --ros-args -p max_linear_vel:=0.5 -p deadzone:=0.15
```

## Dependencies

| Dependency | Description |
|-----------|-------------|
| `rclpy` | ROS 2 Python client library |
| `geometry_msgs` | Twist message type |
| `sensor_msgs` | Joy message type (joystick) |
| `antbot_interfaces` | CargoCommand, WiperOperation service types |
| `std_srvs` | SetBool service type (headlight) |
| `joy` | Joystick driver node (runtime) |

## Build

```bash
colcon build --symlink-install --packages-select antbot_teleop
```

## License

Apache License 2.0 — Copyright 2026 ROBOTIS AI CO., LTD.
