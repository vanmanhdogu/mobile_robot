# ANTBot — Hardware Reference (`hw_info.md`)

Hardware facts for the **real** ANTBot as configured in this workspace, for a
unit with the **3D LiDAR, GNSS, camera and headlight removed**.

Every number below is traced to a file in `src/antbot/` (path and line given).
Where the workspace contradicts itself, the contradiction is written down rather
than resolved — see [§10 Discrepancies](#10-discrepancies-found-in-the-workspace).

> **Not verified against the board.** At the time of writing this machine has no
> `/dev/ttyUSB*`, no `/dev/input/js*`, and is an **NVIDIA Jetson AGX Thor Dev Kit
> running ROS 2 Jazzy** (`/opt/ros/jazzy`) — not the robot's own computer. The
> robot runs ROS 2 Humble inside the container defined in `docker/`. Values
> marked **[measure]** must be read off the real RCU before you trust them.

---

## 1. Platform at a glance

| Item | Value | Source |
|---|---|---|
| Drive type | 4-wheel independent-steering swerve (non-coaxial) | `antbot_description/urdf/antbot.xacro` |
| Dimensions | L762 × W582 × H766 mm (H1373 with flag) | `docs/wiki/.../hardware/specifications.mdx` |
| Mass | 62.1 kg (URDF models 50 kg base + 4×2.5 kg modules) | spec sheet / `base.xacro:10` |
| Payload | 10 kg recommended, 20 kg max | spec sheet |
| Max speed (recommended) | 2.0 m/s | spec sheet |
| Gradeability | 15° | spec sheet |
| Step clearance | 90 mm @5 kg, 50 mm @20 kg | spec sheet |
| Ingress | IPX4, −10…40 °C | spec sheet |
| Control loop | **20 Hz** | `antbot_swerve_controller/config/swerve_drive_controller.yaml:4` |

**Onboard computer (robot):** Jetson AGX Orin 32 GB, Ubuntu 22.04, JetPack 6.0,
ROS 2 Humble, FastDDS, 512 GB NVMe (`docs/wiki/.../hardware/onboard-computer.mdx`).

---

## 2. Subsystems present on this unit

| Subsystem | Interface | Launch file |
|---|---|---|
| ANT-RCU control board | `/dev/ttyUSB0` @ 4 Mbaud, id 200, protocol 2.0 | `antbot_bringup/launch/controller.launch.py` |
| IMU board | `/dev/ttyUSB1` @ 4 Mbaud, id 200 | `antbot_imu/launch/imu.launch.py` |
| 2D LiDAR front (`lidar_0`) | `/dev/ttyUSB3` → frame `lidar_2d_front_link` | `antbot_bringup/launch/lidar_2d.launch.py:39` |
| 2D LiDAR back (`lidar_1`) | `/dev/ttyUSB2` → frame `lidar_2d_back_link` | same file, line 42 |
| Gamepad | `/dev/input/js0` | `antbot_teleop/launch/teleop_joy.launch.py` |

**Removed:** 3D LiDAR (Vanjee), GNSS (u-blox), cameras (Orbbec), headlight.

Consequences of the removals:

* `antbot_bringup/launch/bringup.launch.py:53-65` still includes `lidar_3d`,
  `ublox_gps` and `camera`. **Do not run it as-is** — launch the pieces you have.
* The URDF still publishes the `lidar_3d_*`, `gnss_link` and `camera_*` frames
  (`antbot.xacro:57-92`). Harmless (fixed joints, no data), but TF will show
  frames for hardware that is not there.
* **The headlight is a software device, not a plugin.**
  `board_interface.cpp:86` constructs it unconditionally, so
  `/headlight/operation` still exists and still writes RCU register 334
  (`Headlight_State`). With the lamp unplugged the service returns
  `success: true` anyway — it reports only that the *register write* landed
  (`device/headlight.cpp:36`). Do not use it as a lamp-present test.

---

## 3. Wheels and geometry

From `antbot_description/urdf/antbot.xacro:19-24`:

| Property | Value |
|---|---|
| `wheel_radius` | **0.103 m** |
| `wheelbase_length` (front↔rear steering axes) | 0.530 m → module x = **±0.265 m** |
| `steering_width` (left↔right steering axes) | 0.401 m → module y = **±0.2005 m** |
| `wheelbase_width` (left↔right wheel centres) | 0.512 m → wheel y = ±0.256 m |
| `wheel_offset` (steering axis → wheel centre) | `(0.512 − 0.401)/2` = **0.0555 m** |
| `steering_height` (base_link → steering joint z) | 0.120 m |

The wheel sits **0.0555 m outboard of its own steering axis** — this is what
makes the drive *non-coaxial*, and it is why the controller carries
`steering_to_wheel_y_offsets`.

Joint tree per module (`wheel.xacro`):

```
base_link
  └── steering_<pos>_joint   revolute, axis Z, at (x, y, 0.120)
        └── <pos>_steering_link
              └── wheel_<pos>_joint   continuous, axis Y, at (0, ±0.0555, 0)
                    └── <pos>_wheel_link
```

Module order is fixed everywhere in the stack as
**`front_left, front_right, rear_left, rear_right`** = motors **M1, M2, M3, M4**
and steering **S1, S2, S3, S4** (`device/wheel.cpp:34-46`, `device/steering.cpp:58-71`).

---

## 4. Motors

### 4.1 Drive motors (M1–M4)

| Property | Value | Source |
|---|---|---|
| Command register | `M1..M4_Goal_RPM`, addr 41/45/49/53, 4 B, RW | `control_table.xml` |
| Register scale | 0.01 rev/min per LSB (i.e. raw = rpm × 100) | `constants.hpp:47-48` |
| Register limits | ±18500 raw = **±185 rev/min** | `control_table.xml` |
| Feedback | `M*_Present_RPM` (57…), `M*_Present_Position` (73…), `M*_Present_Current` (89…, mA) | |
| Health | `M*_HW_ERROR` (105-108), `M*_Driver_Temperature` (109-112), `M*_Motor_Temperature` (113-116) | |
| Shared acceleration | `Motor_Goal_Acceleration` addr 36, **53.644 rev/min² per LSB** | `constants.hpp:57` |
| Software velocity clamp | **±20.0 rad/s** | `antbot_hw_interface/config/board_params.yaml:24-25` |

Sanity check: 185 rev/min = 19.37 rad/s; × 0.103 m = **1.995 m/s**, which matches
the 2.0 m/s spec. So the RCU's RPM register is **wheel** RPM — any gearbox is
already accounted for on the board side. There is no drive gear ratio in this
codebase.

`M*_HW_ERROR` bit labels (`control_table.xml:105`):
`No_Error, Over_Volt, Low_Volt, Hot_Inverter, Hot_Motor, Overload, –, Inverter,
–, –, –, –, –, Encoder, Hall_Sensor, Calibration, STO, –, BUS_WDG, Over_Speed`.

**Command path** (`device/wheel.cpp:64-96`):

```
cmd [rad/s] → clamp(±20) → ×60/2π → (int32 truncation) → ×100 → M1_Goal_RPM..M4 (batch write)
```

### 4.2 Steering motors (S1–S4)

Dynamixel-class actuators behind the RCU.

| Property | Value | Source |
|---|---|---|
| Command | `S1..S4_Goal_Position`, addr 122/126/130/134, pulses | `control_table.xml` |
| Resolution | `π/2048` rad per pulse = 4096 pulses/rev | `constants.hpp:43` |
| Zero offset | **2048 pulses** = 0 rad | `constants.hpp:44` |
| **Gear ratio** | **2.43** (motor rev per joint rev) | `board_params.yaml:7` |
| Joint travel limit | **±55°** = ±0.9599 rad, all four modules | `board_params.yaml:8-15` |
| Profile velocity | `S*_Profile_Velocity`, 0.229 rev/min per LSB | `constants.hpp:66` |
| Profile acceleration | `S*_Profile_Acceleration`, 214.577 rev/min² per LSB | `constants.hpp:61` |
| Feedback | `S*_Present_Position` (138…), `S*_Present_Current` (186…), `S*_Motor_Temperature` (202-205) | |
| Errors | `S*_Error_Code` (118-121) | |
| Homing | `Swerve_Homing_Command` addr 117 — `0 None, 1 Auto, 2 Manual_Start, 3 Manual_Done` | |

**Conversions** (`device/steering.cpp:74-83`, `:105-140`):

```
read:   joint_rad = (pulse − 2048) × (π/2048) / 2.43
write:  pulse     = clamp(cmd_rad, ±0.9599) × 2.43 / (π/2048) + 2048
```

Pulse range for ±55°: 0.9599 × 2.43 = 2.3326 rad → ±1521 pulses → **527 … 3569**.
**[measure]** Confirm the physical stops agree before commanding near the limit.

---

## 5. Encoders

`device/encoder.cpp` turns the four `M*_Present_Position` registers into
accumulated `wheel_*_joint/position` state interfaces (which is what reaches
`/joint_states`).

| Property | Value |
|---|---|
| Resolution constant | `TICK_TO_RAD = π/8192` → **16384 ticks/rev** |
| Physical basis | 12-bit encoder (4096) with 4× interpolation (`constants.hpp:40`) |
| Distance per tick | `0.103 × π/8192` = **39.5 µm** at the tread |
| Wrap handling | signed 32-bit wrap detected and corrected (`encoder.cpp:83-110`) |
| Accumulator | `int64_t`, starts at **0** every launch — position is relative, never absolute |

Two gates decide whether ticks accumulate:

1. `Motor_Reboot_Check` (addr 35) ≠ 0 → a motor rebooted. `update()` re-latches
   the raw tick, sets a `was_rebooted_` flag and **returns early without touching
   the accumulator** (`encoder.cpp:52-63`). On the next cycle in which the flag is
   set, `delta` is held at 0 so the reboot cannot inject a huge jump
   (`encoder.cpp:74-81`). `BoardInterface::write()` acknowledges the reboot by
   writing the same value back to the register (`board_interface.cpp:258-269`).
2. `Motor_State` (addr 34) must equal **2 = READY**. Anything else logs
   `Failed to get present position, check motor state : N` every 3 s and the
   position simply freezes (`encoder.cpp:113-121`).

So: **`/joint_states` wheel positions stuck at a constant while the robot moves
means `Motor_State != 2`, not an encoder fault.** Read addr 34 to see which state.

`Motor_State` labels from the control table:
`0 IDLE, 1 READY_ENTER, 2 READY, 3 FAULT_ENTER, 4 FAULT, 5 FAULT_EXIT,
6 NOT_CONNECT, 7 BRAKE`.

---

## 6. Kinematic model

Implemented in `antbot_swerve_controller`. Configured values
(`config/swerve_drive_controller.yaml:16-31`):

```yaml
wheel_radius:                0.103
module_x_offsets:          [ 0.265,  0.265, -0.265, -0.265]
module_y_offsets:          [ 0.256, -0.256,  0.256, -0.256]
module_angle_offsets:      [ 0.0, 0.0, 0.0, 0.0]
module_steering_limit_*:   ∓0.959 rad   (= ∓55°, matches board_params)
steering_to_wheel_y_offsets: [ 0.0555, -0.0555, 0.0555, -0.0555]
module_wheel_speed_limit_*:  ∓50.0 rad/s
```

### 6.1 Inverse kinematics

`swerve_motion_control.cpp:388-420`. For module *i* at body position
(xᵢ, yᵢ) with steering→wheel offset dᵢ, and body twist (vₓ, v_y, ω):

```
pivot velocity at the steering axis
  vpᵢ,ₓ = vₓ − ω·yᵢ
  vpᵢ,y = v_y + ω·xᵢ

steering angle (first estimate)
  θᵢ = atan2(vpᵢ,y , vpᵢ,ₓ)

non-coaxial correction — the wheel is dᵢ away from the axis it pivots about
  vcᵢ,ₓ = vpᵢ,ₓ − ω·dᵢ·cos θᵢ
  vcᵢ,y = vpᵢ,y + ω·dᵢ·sin θᵢ

outputs
  wheel speed  ωwᵢ = ‖vcᵢ‖ / r
  joint angle  θjᵢ = normalize(θᵢ − module_angle_offsetᵢ)
```

`non_coaxial_ik_iterations` (default **0**) adds fixed-point refinement of θᵢ
*before* the correction; the correction itself is applied unconditionally.

**Steering flip:** if both θjᵢ and θjᵢ+π are inside the module limits, the
controller picks whichever is closer to the current angle and negates the wheel
speed accordingly (`enabled_steering_flip`, default true). With ±55° limits the
flipped solution is almost never legal, so on this robot flip rarely fires — the
±55° range, not the flip, is what forbids true omnidirectional strafing.

### 6.2 Forward kinematics / odometry

`odometry.cpp`. The 8 measured module states (4 angles + 4 speeds) over-determine
the 3-DOF body twist, so it is solved as a least-squares problem:

| Parameter | Default | Options |
|---|---|---|
| `odom_solver_method` | `svd` | `pseudo_inverse`, `qr`, `svd` |
| `odom_integration_method` | `rk4` | `euler`, `rk2`, `rk4`, `analytic_swerve` |
| `velocity_rolling_window_size` | 1 | |
| `enable_odom_tf` | true | publishes `odom → base_link` |

Output: `/odom` (`nav_msgs/Odometry`), frames `odom` / `base_link`,
pose & twist covariance diagonals 0.001.

### 6.3 Speed limits

`enabled_speed_limits` defaults to **false**
(`swerve_drive_controller_parameter.yaml:5`), so the `linear.x ±2.0`,
`linear.y ±1.5`, `angular.z ±2.0` limits in that file are **inactive** unless
you turn the flag on. The limits that always apply are the per-module
`module_wheel_speed_limit_*` (±50 rad/s) and, further down, the hardware
interface's own ±20 rad/s clamp.

---

## 7. ANT-RCU control board

| Property | Value |
|---|---|
| Device name | `ANTBOT-RCU` |
| Model number | **526** |
| Protocol | Dynamixel 2.0 |
| Bus id | 200 |
| Port / baud | `/dev/ttyUSB0` @ **4 000 000** |
| Address span | `MinAddress 0` … `MaxAddress 377` |
| Control table | `antbot_hw_interface/config/control_table.xml` |

The whole table is re-read **every control cycle** (`board_interface.cpp:211`),
so `/hw/direct_read` always returns fresh values.

### 7.1 Register map by area

| Range | Area |
|---|---|
| 0–10 | Identity: model, generation, board & firmware version, bus id, baud, `Error_Code` |
| 17–30 | System: `Operation_Command` (1 = reboot), `Sec_Since_Power_On`, `System_State`, `Estop_State`, `Remote_Connected` |
| 34–116 | Drive motors: state, goal/present RPM, position, current, HW error, temperatures |
| 117–205 | Steering: homing, error codes, goal/present position, profile vel/accel, current, temperature |
| 206–233 | Battery & BMS (§8) |
| 234–247 | Docking & charging (§9) |
| 248–274 | Voltage rails: `Voltage_Batt`, `Voltage_RCU_{14V,12V,12V_2,5V,3V3}`, `Voltage_ORIN_{17V,12V,5V,3V3,1V8,1V2,LIDAR,ROUTER}` — all 0.1 V/LSB |
| 276–290 | Currents: `Current_{Batt,Motor,Charge,RCU_12V,RCU_12V_2,RCU_5V,HEAT_POWER,ORIN_Batt}` — 0.1 A/LSB |
| 292–296 | Cargo: `Module_Type`, `Cargo_Command`, `Cargo_Door_State`, `Cargo_Lock_State`, `Cargo_Detect_Adc` |
| 298–308 | Wipers ×3: mode, interval, connected, pose, HW error |
| 309–334 | LEDs (flag / indicators / eye / turn signal) and `Headlight_State` |
| 335–347 | Fans ×5, relays ×8 + relay timeout |
| 348–355 | Pressure sensor, `ATM_Pressure` |
| 356–359 | `UltraSonic_1`, `UltraSonic_2` (cm) |
| 360–362 | `Orin_Power_Control` (1 off / 2 on / 3 reboot), `Orin_Power_State`, `ESP32_Connection` |
| 363–376 | Magnetometer: connection, temperature, scale, timestamp, X/Y/Z |
| 800 | `IMU_Sensor_Connection` — **outside MaxAddress 377, see §10.5** |

`System_State` labels: `0 START, 1 CHECKING, 2 READY, 3 RUNNING, 4 ERROR,
5 ESTOP, 6 IDLE`. `Estop_State`: `0 OFF, 1 ON`.

### 7.2 Key system registers worth watching

```
Error_Code (10)   System_State (27)   Estop_State (28)   Motor_State (34)
Sec_Since_Power_On (19)   Remote_Connected (29)   Operation_Busy (18)
```

---

## 8. Battery

| Property | Value | Source |
|---|---|---|
| Chemistry | LFP, 1 pack | spec sheet |
| Capacity | **65 Ah**, nominal **25.6 V** | spec sheet |
| Runtime / charge time | 8 h / 8 h | spec sheet |
| Low-battery behaviour | below 5 %: 1 min buzzer, then power off | `docs/wiki/.../battery-charging.mdx` |

### 8.1 Registers

| Addr | Item | Scale |
|---|---|---|
| 206 | `Battery_Is_Charging` | `0 Discharging, 1 Charging` |
| 207 | `Battery_Voltage` | 0.01 V |
| 209 | `Battery_Current` | 10 mA, **signed int16** |
| 211 | `Battery_Percentage` | 1 % |
| 212 | `Battery_Capacity` | 0.01 Ah |
| 214 | `Battery_Charge_Current` | 0.01 A |
| 216 | `BMS_Is_Connected` | `0 Not connected, 1 Connected` |
| 217 | `BMS_SOC` | 1 % |
| 218 / 222 | `BMS_Status`, `BMS_LOG` | raw |
| 224 / 226 | `BMS_Voltage` (0.01 V), `BMS_Current` (10 mA, signed) | |
| 228 / 229 | `BMS_Temperature`, `BMS_Temperature_2` | °C, **signed int8** |
| 230–233 | `BMS_Heater_Enable`, `BMS_Heater_Is_On`, `BMS_Heater_Mode` (`0 Off, 1 Auto, 2 Manual`), `BMS_Heater_Manual_On` | |

### 8.2 The `/battery` topic

Published by the `board_hw_interface` node at the 20 Hz control rate,
`sensor_msgs/BatteryState`, best-effort QoS (`device/battery.cpp:27`):

| Field | Mapping |
|---|---|
| `voltage` | `Battery_Voltage` × 0.01 → V |
| `current` | `Battery_Current` × 10 × 0.001 → A |
| `capacity` | `Battery_Capacity` × 0.01 → Ah |
| `percentage` | `Battery_Percentage` × 0.01 → **ratio 0.0–1.0** |
| `temperature` | `BMS_Temperature` → °C |
| `power_supply_status` | **only 1 (CHARGING) or 3 (NOT_CHARGING)** |
| `power_supply_health`, `power_supply_technology` | hardcoded `UNKNOWN` — ignore |

`DISCHARGING (2)` and `FULL (4)` are never published (`device/battery.cpp:43-47`),
so **status 3 is the normal running state, not a fault.** Use `percentage` for
charge level and `Charge_Sequence` (§9) for charge progress.

---

## 9. Docking and charging

**There is no autonomous docking node, action or service in this workspace.**
Docking is physical: reverse the robot into the wireless station and charging
starts automatically (`docs/wiki/.../battery-charging.mdx`). What the software
offers is telemetry plus a manual override.

The charging coil frame is `robot_charging_coil_link` at
`(−0.299, 0.0, 0.171)` on `base_link` (`antbot.xacro:95`) — i.e. **rear-centre**,
which is why docking is a reverse manoeuvre.

| Addr | Item | Access | Values |
|---|---|---|---|
| 234 | `Docking` | R | `0 No, 1 Yes` |
| 235 | `Charge_Error` | R | error code |
| 236 | `Charge_Sequence` | R | `0 Standby, 1 Optimize Current, 2 On Charge, 3 Full Charged, 4 Charge Error, 5 Stop Charge` |
| 237 | `Charge_Command` | **R/W** | `0 None, 1 Grip, 2 Release, 3 Force Charge Enable, 4 Force Charge Disable, 5 Force Charge Clear` |
| 238 | `Charge_Swerve_Status` | R | `0 Released, 1 Gripped` |
| 239 | `Charge_Force_Status` | R | `0 None, 1 Force Enabled, 2 Force Disabled` |
| 240 / 242 | `Charge_Hall_Left` / `_Right` | R | hall raw — alignment feedback |
| 244 / 245 | `Charging_Station_ID` / `_SET_ID` | R / **R/W** | station id |
| 246 / 247 | `Charge_FW_Version_Major` / `_Minor` | R | charger firmware |

A healthy dock walks `Charge_Sequence` `0 → 1 → 2` with `Docking = 1`,
`Charge_Swerve_Status = 1`, and `/battery.power_supply_status` flipping 3 → 1.

> ⚠️ `Charge_Command` 1/2 (Grip/Release) moves **real hardware**, and
> `/hw/direct_write` does **no range or safety validation** — the value goes
> straight to the register (`board_interface.cpp:89-99`). Only grip while
> actually docked. Use Force Charge for diagnostics only and clear it with `5`.

---

## 10. Discrepancies found in the workspace

These are stated as found, not fixed. Each one is worth settling against the
real robot before it bites.

### 10.1 `module_y_offsets` may double-count the steering→wheel offset

The controller documents `module_y_offsets` as *"y-distance from base_link to
**steering joint**"* and sets it to **±0.256 m**. But the URDF puts the steering
joints at `±steering_width/2` = **±0.2005 m**, and ±0.256 m is the **wheel**
centre (`±wheelbase_width/2`). Note that `0.2005 + 0.0555 = 0.256`, and
`steering_to_wheel_y_offsets` is *separately* configured as ±0.0555 m and added
on top inside the IK (`swerve_motion_control.cpp:404-419`).

If the controller's field really means the steering axis, the effective lever arm
during rotation becomes 0.3115 m instead of 0.256 m — a ~22 % error that would
show as heading drift when spinning in place.

**[measure]** Spin the robot 10 full turns in place on the ground with
`enable_odom_tf` on and compare `/odom` yaw against reality. If yaw is
over-reported, try `module_y_offsets: [0.2005, -0.2005, 0.2005, -0.2005]`.

### 10.2 Velocity clamp exceeds the register limit

`board_params.yaml` clamps wheel commands to **±20.0 rad/s**, which converts to
190.99 rev/min → raw **19099**. The `M*_Goal_RPM` register maximum is **18500**
(±185 rev/min). Commands between 185 and 191 rev/min are out of range; what the
board does with them is firmware-defined. **[measure]** — or lower the clamp to
19.37 rad/s, which is exactly 185 rev/min.

### 10.3 Three `board_params.yaml` keys are never read

`wheel.accel`, `wheel.min_rpm` and `wheel.max_rpm` are in the YAML but
`device/wheel.cpp:28-32` only declares `wheel.min_velocity` and
`wheel.max_velocity`. Wheel acceleration actually comes from the controller's
`wheel.max_acceleration` (18.7254 rad/s²) through the `acceleration` command
interface. Editing `wheel.accel` changes nothing.

### 10.4 `Motor_State` constants disagree with the control table

`constants.hpp:69-74` defines `MOTOR_RUNNING=3, MOTOR_FAULT=4, MOTOR_BRAKE=5`,
but the table labels 3/4/5 as `FAULT_ENTER / FAULT / FAULT_EXIT` (BRAKE is 7).
Only `MOTOR_READY=2` is used in code, and that one is correct — so this is
currently a documentation hazard, not a runtime bug. Trust the control table.

### 10.5 `IMU_Sensor_Connection` cannot be read through this table

It sits at address **800** while the device declares `MaxAddress="377"`.
`Communicator::get_data` rejects anything past `max_address_`
(`communicator.hpp:70-75`), so `/hw/direct_read` on that name logs an address
error and returns 0. Check the IMU via its own node on `/dev/ttyUSB1` instead.

### 10.6 `/hw/direct_read` returns signed registers as unsigned

`get_data` copies only the register's own length into a zeroed `int32_t`
(`communicator.hpp:81-98`), and the service returns that as `int32`
(`board_interface.cpp:102-112`). For **signed** registers the sign bit is never
extended:

| Register | True −1 reads back as |
|---|---|
| `BMS_Temperature` (int8) | **255** |
| `Battery_Current` (int16) | **65535** |
| `M*_Motor_Temperature` (int8) | **255** |

Rule of thumb: for a 1-byte signed register, subtract 256 if the value > 127;
for 2-byte, subtract 65536 if > 32767. The `/battery` topic does **not** have
this problem — the device classes read with the correct signed type
(`device/battery.cpp:31-36`).

### 10.7 Scale factors are not applied by `direct_read`

The service returns the **raw** register. Apply the `Scale` attribute from
`control_table.xml` yourself: `Battery_Voltage` × 0.01 → V,
`Voltage_*` × 0.1 → V, `Current_*` × 0.1 → A, `M*_Present_RPM` × 0.01 → rev/min.

### 10.8 URDF steering limits are wider than the real ones

`wheel.xacro` declares `lower="-1.5708" upper="1.5708"` (±90°) on the steering
joints, while the board and the controller both enforce **±55°**. RViz and any
planner reading the URDF will believe in travel the robot does not have.

### 10.9 Ultrasound topic name

The package README calls it `sensor/ultrasound`; the source publishes
**`ultrasound`** (`device/ultrasound.cpp:32`). Trust the source.

---

## 11. ROS 2 surface of the hardware interface

Plugin `antbot/hw_interface/BoardInterface`, component name **`ant_hardware`**,
internal node **`board_hw_interface`** (`board_interface.cpp:55`).

> `board_params.yaml` is keyed `board_hw_interface:`, **not** `controller_manager:`.
> That key must stay exactly as it is or the parameters silently fail to load.

### 11.1 Interfaces exported

Per wheel joint (×4): command `velocity`, `acceleration`; state `velocity`,
`effort`, `position`.
Per steering joint (×4): command `position`, `velocity`, `acceleration`;
state `position`, `effort`.
(`antbot_description/urdf/ros2_control.xacro`)

`effort` is **current in amperes**, not torque — `M*_Present_Current` (mA) × 0.001
(`device/wheel.cpp:58-63`, `device/steering.cpp:85-90`).

### 11.2 Topics

| Topic | Type | Published by |
|---|---|---|
| `/battery` | `sensor_msgs/BatteryState` | hw interface |
| `/ultrasound` | `std_msgs/Float64MultiArray` (2 values, metres) | hw interface |
| `/cargo/status` | `antbot_interfaces/CargoStatus` | hw interface |
| `/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster |
| `/odom` | `nav_msgs/Odometry` | swerve controller |
| `/limited_cmd_vel` | `geometry_msgs/Twist` | swerve controller |
| `/antbot_swerve_controller/planned_trajectory` | `trajectory_msgs/JointTrajectory` | swerve controller |
| `/cmd_vel` | `geometry_msgs/Twist` | **subscribed** by the controller |

### 11.3 Services

| Service | Type | Notes |
|---|---|---|
| `/hw/direct_read` | `antbot_interfaces/srv/DirectRead` | `{item_name} → {data, message}`; `message` echoes `addr[N]`, or `item not found` |
| `/hw/direct_write` | `antbot_interfaces/srv/DirectWrite` | `{item_name, data} → {success, message}`; **no validation** |
| `/cargo/command` | `antbot_interfaces/srv/CargoCommand` | `OPERATION_LOCK=0`, `OPERATION_UNLOCK=1` |
| `/headlight/operation` | `std_srvs/srv/SetBool` | writes reg 334 even with the lamp removed |
| `/wiper/operation` | `antbot_interfaces/srv/WiperOperation` | `OFF=0`, `ONCE=1`, `REPEAT=2` |

`/cargo/command`, `/headlight/operation` and `/wiper/operation` are created in
`activate()` and destroyed in `deactivate()` — they exist **only while the
hardware component is active**. `/hw/direct_read` and `/hw/direct_write` are
created in `on_init()` and are available as soon as the node is up.

---

## 12. Quick CLI reference

Full procedures are in `real_robot_checking.md`; scripted versions are in
`tools/antbot_check.sh` and `tools/antbot_dock.sh`.

```bash
# --- lifecycle -------------------------------------------------------------
ros2 control list_hardware_components        # ant_hardware must be 'active'
ros2 control list_hardware_interfaces
ros2 control list_controllers                # both must be 'active'

# --- telemetry -------------------------------------------------------------
ros2 topic hz   /joint_states                # expect ~20 Hz
ros2 topic echo /battery --once
ros2 topic echo /ultrasound --once
ros2 topic echo /odom --once

# --- raw registers ---------------------------------------------------------
ros2 service call /hw/direct_read antbot_interfaces/srv/DirectRead \
  "{item_name: 'Motor_State'}"
ros2 service call /hw/direct_write antbot_interfaces/srv/DirectWrite \
  "{item_name: 'Charge_Command', data: 2}"

# --- scripted checks -------------------------------------------------------
./tools/antbot_check.sh all                  # every check below, in order
./tools/antbot_check.sh ports hw controllers battery encoder motors power docking
./tools/antbot_dock.sh monitor               # live docking/charge state
./tools/antbot_dock.sh release               # ungrip the charging contacts
```
