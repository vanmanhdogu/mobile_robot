# Copyright 2026 ROBOTIS AI CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Holonomic joystick teleop for AntBot using a Logitech F710 gamepad.

Implements the control table documented in ``joystick.md``: RB is a dead-man
switch, the left stick drives forward/backward, the right stick rotates in
place, the D-pad adds fixed-step forward/backward and holonomic strafing, and
LB / LT / Y / A pick the speed level.

The pad must be in XInput mode (rear D/X switch on X), which enumerates 8 axes
and 11 buttons.
"""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

try:
    from antbot_interfaces.srv import CargoCommand
    from antbot_interfaces.srv import WiperOperation
    from std_srvs.srv import SetBool
    ACCESSORY_SRVS_AVAILABLE = True
except ImportError:  # antbot_interfaces not built -- drive-only mode
    ACCESSORY_SRVS_AVAILABLE = False

# Which axis pair carries the analog left stick and which carries the digital
# D-pad. 'standard' is what an F710 in XInput mode reports through SDL: the
# stick is SDL axes 0/1 and the hat is appended as axes 6/7. 'mode_swapped' is
# the layout described in joystick.md section 2.1, which is what the pad
# reports once the MODE button has swapped the stick and the D-pad. See
# _check_axis_profile() for the runtime cross-check.
AXIS_PROFILES = {
    'standard': {'stick': (0, 1), 'dpad': (6, 7)},
    'mode_swapped': {'stick': (6, 7), 'dpad': (0, 1)},
}

MIN_AXES = 8
MIN_BUTTONS = 11

SPEED_LEVEL_MIN = 1
SPEED_LEVEL_MAX = 9

# A hat only ever reports -1.0, 0.0 or +1.0. Anything in between on the pair we
# believe is the D-pad proves the profile is wrong.
DIGITAL_EPS = 1e-3

# Frames of stick deflection without a single fractional reading before we
# suspect the MODE button has turned the left stick into a digital control.
MODE_SUSPECT_FRAMES = 60


def clamp(value, limit):
    return max(-limit, min(limit, value))


class TeleopJoystickF710Node(Node):

    def __init__(self):
        super().__init__('teleop_joystick_f710')

        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('speed_level', 3)
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('dpad_gain', 0.5)
        self.declare_parameter('lt_threshold', -0.5)
        self.declare_parameter('watchdog_timeout', 0.3)
        self.declare_parameter('zero_repeat', 5)
        self.declare_parameter('axis_profile', 'standard')
        self.declare_parameter('enable_accessories', True)

        # Axis indices that do not move between profiles.
        self.declare_parameter('axis_lt', 2)
        self.declare_parameter('axis_rt', 5)
        self.declare_parameter('axis_rstick_x', 3)

        # Button indices (XInput order).
        self.declare_parameter('btn_deadman', 5)        # RB
        self.declare_parameter('btn_force_slow', 4)     # LB
        self.declare_parameter('btn_speed_up', 3)       # Y
        self.declare_parameter('btn_speed_down', 0)     # A
        self.declare_parameter('btn_cargo_lock', 2)     # X
        self.declare_parameter('btn_cargo_unlock', 1)   # B
        self.declare_parameter('btn_headlight', 6)      # BACK
        self.declare_parameter('btn_wiper', 7)          # START

        self.max_linear_vel = self._p('max_linear_vel').double_value
        self.max_angular_vel = self._p('max_angular_vel').double_value
        self.speed_level = self._clamp_level(self._p('speed_level').integer_value)
        self.deadzone = self._p('deadzone').double_value
        self.dpad_gain = self._p('dpad_gain').double_value
        self.lt_threshold = self._p('lt_threshold').double_value
        self.watchdog_timeout = self._p('watchdog_timeout').double_value
        self.zero_repeat = self._p('zero_repeat').integer_value
        self.enable_accessories = self._p('enable_accessories').bool_value

        self.axis_lt = self._p('axis_lt').integer_value
        self.axis_rt = self._p('axis_rt').integer_value
        self.axis_rstick_x = self._p('axis_rstick_x').integer_value

        self.btn = {
            key: self._p(key).integer_value
            for key in ('btn_deadman', 'btn_force_slow',
                        'btn_speed_up', 'btn_speed_down',
                        'btn_cargo_lock', 'btn_cargo_unlock',
                        'btn_headlight', 'btn_wiper')
        }

        profile_name = self._p('axis_profile').string_value
        if profile_name not in AXIS_PROFILES:
            self.get_logger().warn(
                f"Unknown axis_profile '{profile_name}', "
                f"falling back to 'standard'")
            profile_name = 'standard'
        self._set_profile(profile_name, announce=False)

        # Edge detection, watchdog and warn-once state.
        self.prev_buttons = {}
        self.last_joy_time = None
        self.was_driving = False
        self.zero_frames_left = 0
        self.deadman_held = False
        self.analog_seen = False
        self.digital_stick_frames = 0
        self.mode_warned = False
        self.short_msg_warned = False
        self.headlight_on = False
        self.wiper_on = False

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

        self.cargo_client = None
        self.headlight_client = None
        self.wiper_client = None
        if self.enable_accessories and ACCESSORY_SRVS_AVAILABLE:
            try:
                self.cargo_client = self.create_client(
                    CargoCommand, 'cargo/command')
                self.headlight_client = self.create_client(
                    SetBool, 'headlight/operation')
                self.wiper_client = self.create_client(
                    WiperOperation, 'wiper/operation')
            except Exception as e:  # noqa: BLE001 - stale/ABI-mismatched msgs
                # antbot_interfaces built against a different ROS distro or
                # Python version loads as a module but has no usable type
                # support. Driving must not be lost over an accessory button.
                self.cargo_client = None
                self.headlight_client = None
                self.wiper_client = None
                self.get_logger().error(
                    f'Could not create accessory service clients ({e}). '
                    f'Cargo / headlight / wiper buttons are disabled; '
                    f'driving is unaffected. Rebuild antbot_interfaces for '
                    f'this ROS distro to re-enable them.')
        elif self.enable_accessories:
            self.get_logger().warn(
                'antbot_interfaces not importable, '
                'cargo / headlight / wiper buttons are disabled')

        # Watchdog runs faster than the timeout so a dropped 2.4 GHz link is
        # caught promptly (joystick.md section 5.3).
        self.watchdog_timer = self.create_timer(
            max(self.watchdog_timeout / 3.0, 0.02), self._watchdog)

        self._print_help()

    # -- parameter / helper plumbing -------------------------------------

    def _p(self, name):
        return self.get_parameter(name).get_parameter_value()

    @staticmethod
    def _clamp_level(level):
        return max(SPEED_LEVEL_MIN, min(SPEED_LEVEL_MAX, level))

    def _set_profile(self, name, announce=True):
        self.profile_name = name
        self.axis_stick_y = AXIS_PROFILES[name]['stick'][1]
        self.axis_dpad_x, self.axis_dpad_y = AXIS_PROFILES[name]['dpad']
        if announce:
            self.get_logger().warn(f"Axis profile switched to '{name}'")

    def _deadzone(self, value):
        """Deadzone with rescale so the output has no step at the edge."""
        if abs(value) < self.deadzone:
            return 0.0
        span = 1.0 - self.deadzone
        if span <= 0.0:
            return math.copysign(1.0, value)
        return math.copysign((abs(value) - self.deadzone) / span, value)

    def _pressed(self, msg, key):
        """True on the 0 -> 1 rising edge (joystick.md section 5.2)."""
        idx = self.btn[key]
        curr = msg.buttons[idx]
        prev = self.prev_buttons.get(key, 0)
        self.prev_buttons[key] = curr
        return curr == 1 and prev == 0

    def _print_help(self):
        self.get_logger().info(
            '\n'
            '--- Logitech F710 (XInput) Teleop ---\n'
            'RB (hold)    : dead-man, robot only moves while held\n'
            'Left stick Y : forward / backward      (linear.x)\n'
            'Right stick X: rotate CCW / CW         (angular.z)\n'
            'D-pad up/down: forward / backward step (linear.x)\n'
            'D-pad l/r    : strafe left / right     (linear.y)\n'
            'LB (hold)    : force speed level 1\n'
            'LT (hold)    : force speed level 9\n'
            'Y / A        : speed level +1 / -1\n'
            'X / B        : cargo lock / unlock\n'
            'BACK / START : headlight / wiper toggle\n'
            f'axis profile : {self.profile_name}\n'
            f'speed level  : {self.speed_level}/9\n'
            '-------------------------------------')

    # -- safety checks ----------------------------------------------------

    def _check_axis_profile(self, msg):
        """Cross-check the stick / D-pad assignment against live values.

        A hat can only ever read -1, 0 or +1, so a fractional value on the pair
        we treat as the D-pad means the assignment is inverted -- swap it. The
        opposite failure, the MODE button turning the left stick into a digital
        control, produces no fractional values anywhere and can only be warned
        about (joystick.md section 5.4).
        """
        dpad_vals = (msg.axes[self.axis_dpad_x], msg.axes[self.axis_dpad_y])
        for value in dpad_vals:
            if DIGITAL_EPS < abs(value) < 1.0 - DIGITAL_EPS:
                other = ('mode_swapped' if self.profile_name == 'standard'
                         else 'standard')
                self.get_logger().error(
                    f'Analog value {value:+.3f} on an axis mapped to the '
                    f'D-pad: the stick and D-pad are not where '
                    f"'{self.profile_name}' expects them.")
                self._set_profile(other)
                self.analog_seen = True
                self.digital_stick_frames = 0
                return

        if self.analog_seen or self.mode_warned:
            return

        stick = msg.axes[self.axis_stick_y]
        if abs(stick) <= DIGITAL_EPS:
            return
        if abs(stick) < 1.0 - DIGITAL_EPS:
            self.analog_seen = True
            return

        # Deflected, but only ever to the rail.
        self.digital_stick_frames += 1
        if self.digital_stick_frames >= MODE_SUSPECT_FRAMES:
            self.mode_warned = True
            self.get_logger().warn(
                'Left stick has only ever reported +/-1.0, never an analog '
                'value. The MODE button may be on (green MODE LED lit), which '
                'swaps the stick and D-pad. Press MODE once to turn it off.')

    # -- accessory services ----------------------------------------------

    def _call_cargo(self, operation):
        if self.cargo_client is None:
            return
        if not self.cargo_client.service_is_ready():
            self.get_logger().warn('cargo/command not available')
            return
        req = CargoCommand.Request()
        req.operation = operation
        self.cargo_client.call_async(req).add_done_callback(self._on_response)
        name = 'LOCK' if operation == CargoCommand.Request.OPERATION_LOCK \
            else 'UNLOCK'
        self.get_logger().info(f'Cargo {name} requested')

    def _toggle_headlight(self):
        if self.headlight_client is None:
            return
        if not self.headlight_client.service_is_ready():
            self.get_logger().warn('headlight/operation not available')
            return
        self.headlight_on = not self.headlight_on
        req = SetBool.Request()
        req.data = self.headlight_on
        self.headlight_client.call_async(req).add_done_callback(
            self._on_response)
        self.get_logger().info(
            f"Headlight {'ON' if self.headlight_on else 'OFF'} requested")

    def _toggle_wiper(self):
        if self.wiper_client is None:
            return
        if not self.wiper_client.service_is_ready():
            self.get_logger().warn('wiper/operation not available')
            return
        self.wiper_on = not self.wiper_on
        req = WiperOperation.Request()
        req.mode = WiperOperation.Request.REPEAT if self.wiper_on \
            else WiperOperation.Request.OFF
        self.wiper_client.call_async(req).add_done_callback(self._on_response)
        self.get_logger().info(
            f"Wiper {'REPEAT' if self.wiper_on else 'OFF'} requested")

    def _on_response(self, future):
        try:
            resp = future.result()
            if not resp.success:
                self.get_logger().warn(f'Service call failed: {resp.message}')
        except Exception as e:  # noqa: BLE001 - report any transport failure
            self.get_logger().warn(f'Service call failed: {e}')

    # -- speed model ------------------------------------------------------

    def _effective_level(self, msg):
        """LB (slowest) beats LT (fastest) beats the Y/A base level."""
        if msg.buttons[self.btn['btn_force_slow']] == 1:
            return SPEED_LEVEL_MIN
        # An untouched trigger can read 0.0 before its first event, so the
        # threshold must sit below 0.0 (joystick.md section 5.1).
        if msg.axes[self.axis_lt] < self.lt_threshold:
            return SPEED_LEVEL_MAX
        return self.speed_level

    # -- main loop --------------------------------------------------------

    def _publish(self, twist):
        self.cmd_vel_pub.publish(twist)

    def _stop(self):
        self._publish(Twist())

    def _watchdog(self):
        """Stop the robot when /joy goes quiet (joystick.md section 5.3)."""
        if self.last_joy_time is None:
            return
        gap = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if gap < self.watchdog_timeout:
            return
        if self.was_driving:
            self.get_logger().warn(f'No /joy for {gap:.2f} s, stopping')
            self.was_driving = False
            self.zero_frames_left = self.zero_repeat
            self.prev_buttons.clear()
            self.deadman_held = False
        # A bounded burst of zeros, not a permanent stream: a disconnected pad
        # must brake the robot without holding /cmd_vel down forever against
        # whatever else may publish on it.
        if self.zero_frames_left > 0:
            self.zero_frames_left -= 1
            self._stop()

    def joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()

        if len(msg.axes) < MIN_AXES or len(msg.buttons) < MIN_BUTTONS:
            if not self.short_msg_warned:
                self.short_msg_warned = True
                self.get_logger().error(
                    f'/joy has {len(msg.axes)} axes and {len(msg.buttons)} '
                    f'buttons, expected at least {MIN_AXES} and '
                    f'{MIN_BUTTONS}. Set the rear D/X switch to X.')
            self._stop()
            return

        self._check_axis_profile(msg)

        # Dead-man switch: nothing moves unless RB is held.
        deadman = msg.buttons[self.btn['btn_deadman']] == 1
        if deadman != self.deadman_held:
            self.deadman_held = deadman
            self.get_logger().info(
                f"Dead-man {'engaged' if deadman else 'released'}")
        if not deadman:
            # Keep edges fresh so a button held across the release does not
            # fire the moment the dead-man is re-engaged.
            for key in self.btn:
                self.prev_buttons[key] = msg.buttons[self.btn[key]]
            if self.was_driving:
                self.was_driving = False
                self.zero_frames_left = self.zero_repeat
            if self.zero_frames_left > 0:
                self.zero_frames_left -= 1
                self._stop()
            return

        # Speed level, latched on the rising edge only (section 5.2).
        if self._pressed(msg, 'btn_speed_up'):
            self.speed_level = self._clamp_level(self.speed_level + 1)
            self.get_logger().info(f'Speed level UP: {self.speed_level}/9')
        if self._pressed(msg, 'btn_speed_down'):
            self.speed_level = self._clamp_level(self.speed_level - 1)
            self.get_logger().info(f'Speed level DOWN: {self.speed_level}/9')

        if self._pressed(msg, 'btn_cargo_lock'):
            self._call_cargo(CargoCommand.Request.OPERATION_LOCK)
        if self._pressed(msg, 'btn_cargo_unlock'):
            self._call_cargo(CargoCommand.Request.OPERATION_UNLOCK)
        if self._pressed(msg, 'btn_headlight'):
            self._toggle_headlight()
        if self._pressed(msg, 'btn_wiper'):
            self._toggle_wiper()

        level = self._effective_level(msg)
        v = self.max_linear_vel * level / SPEED_LEVEL_MAX
        w = self.max_angular_vel * level / SPEED_LEVEL_MAX

        stick_y = self._deadzone(msg.axes[self.axis_stick_y])
        spin_x = self._deadzone(msg.axes[self.axis_rstick_x])
        dpad_x = msg.axes[self.axis_dpad_x]
        dpad_y = msg.axes[self.axis_dpad_y]

        twist = Twist()
        twist.linear.x = clamp(stick_y * v + dpad_y * v * self.dpad_gain, v)
        twist.linear.y = clamp(dpad_x * v * self.dpad_gain, v)
        twist.angular.z = clamp(spin_x * w, w)

        driving = (twist.linear.x != 0.0 or twist.linear.y != 0.0 or
                   twist.angular.z != 0.0)
        if driving:
            self.zero_frames_left = 0
            self._publish(twist)
        elif self.was_driving:
            self.zero_frames_left = self.zero_repeat
            self._stop()
            self.zero_frames_left -= 1
        elif self.zero_frames_left > 0:
            self.zero_frames_left -= 1
            self._stop()
        self.was_driving = driving


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoystickF710Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.context.ok():
            node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
