#!/usr/bin/env python3
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
#
# Author: Jaehong Oh
"""
Compare the slam_toolbox pose estimate against wheel/swerve odometry.

slam_toolbox does estimate the robot pose (LiDAR scan matching against the pose
graph) and exposes it two ways:

  * ``map -> odom`` TF, republished every ``transform_publish_period``
    (0.02 s in the AntBot config).  Chaining it with ``odom -> base_link``
    from the swerve controller gives the SLAM pose of the robot at any time.
    This is the estimate Nav2 and RViz actually use.
  * ``/pose`` (``geometry_msgs/PoseWithCovarianceStamped``, map frame), which
    is *event driven*: slam_toolbox only publishes it when a scan is accepted
    into the pose graph, i.e. after ``minimum_travel_distance`` /
    ``minimum_travel_heading`` have been exceeded and at most every
    ``minimum_time_interval``.  A stationary robot publishes nothing at all.

This node samples both sources at a fixed rate and logs, per sample:
the odometry pose (``odom -> base_link``), the SLAM pose (``map -> base_link``)
and the correction between them (``map -> odom``).  That correction *is* the
accumulated odometry drift that SLAM has removed, so it is the number to watch
when judging how good the swerve odometry is.

Usage::

    # real robot (after bringup + slam.launch.py mode:=real)
    ros2 run semi_humanoid_navigation odom_slam_test.py

    # simulation
    ros2 run semi_humanoid_navigation odom_slam_test.py --ros-args -p use_sim_time:=true

    # custom log file and 60 s run
    ros2 run semi_humanoid_navigation odom_slam_test.py --ros-args \
        -p csv_path:=/tmp/odom_slam.csv -p duration:=60.0
"""

import csv
import math
import os
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros


def yaw_from_quaternion(q):
    """Return the yaw angle (rad) of a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


CSV_HEADER = [
    't',
    'odom_x', 'odom_y', 'odom_yaw_deg',
    'slam_x', 'slam_y', 'slam_yaw_deg',
    'diff_x', 'diff_y', 'diff_dist', 'diff_yaw_deg',
    'corr_x', 'corr_y', 'corr_yaw_deg',
    'odom_path_len', 'slam_path_len',
    'pose_topic_x', 'pose_topic_y', 'pose_topic_yaw_deg', 'pose_topic_age',
]


class OdomSlamTest(Node):

    def __init__(self):
        super().__init__('odom_slam_test')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('slam_pose_topic', '/pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('print_period', 2.0)
        self.declare_parameter('duration', 0.0)
        self.declare_parameter('csv_path', '')

        self.odom_topic_ = self.get_parameter('odom_topic').value
        self.slam_pose_topic_ = self.get_parameter('slam_pose_topic').value
        self.map_frame_ = self.get_parameter('map_frame').value
        self.odom_frame_ = self.get_parameter('odom_frame').value
        self.base_frame_ = self.get_parameter('base_frame').value
        self.rate_ = float(self.get_parameter('rate').value)
        self.print_period_ = float(self.get_parameter('print_period').value)
        self.duration_ = float(self.get_parameter('duration').value)

        csv_path = self.get_parameter('csv_path').value
        if not csv_path:
            csv_path = os.path.join(
                '/tmp', time.strftime('odom_slam_test_%Y%m%d_%H%M%S.csv'))
        self.csv_path_ = csv_path

        # The swerve controller publishes /odom with SensorDataQoS (BEST_EFFORT).
        # A BEST_EFFORT subscription still matches a RELIABLE publisher, so this
        # QoS works for the simulated and the real robot alike.
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # slam_toolbox publishes /pose RELIABLE / VOLATILE.
        pose_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.tf_buffer_ = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        self.odom_msg_ = None
        self.odom_count_ = 0
        self.odom_first_t_ = None
        self.odom_last_t_ = None

        self.pose_msg_ = None
        self.pose_count_ = 0
        self.pose_first_t_ = None
        self.pose_last_t_ = None
        self.pose_intervals_ = []
        self.pose_prev_xy_ = None
        self.pose_steps_ = []

        self.samples_ = 0
        self.tf_failures_ = 0
        self.max_dist_ = 0.0
        self.sum_dist_ = 0.0
        self.max_yaw_ = 0.0
        self.sum_yaw_ = 0.0
        self.last_row_ = None
        self.odom_path_len_ = 0.0
        self.slam_path_len_ = 0.0
        self.prev_odom_xy_ = None
        self.prev_slam_xy_ = None
        self.max_odom_topic_tf_gap_ = 0.0
        self.summary_printed_ = False

        self.create_subscription(
            Odometry, self.odom_topic_, self.odom_callback, odom_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, self.slam_pose_topic_,
            self.pose_callback, pose_qos)

        self.csv_file_ = open(self.csv_path_, 'w', newline='')
        self.csv_writer_ = csv.writer(self.csv_file_)
        self.csv_writer_.writerow(CSV_HEADER)

        self.start_wall_ = time.time()
        self.last_print_ = 0.0
        self.create_timer(1.0 / self.rate_, self.sample)

        self.get_logger().info(
            f'Comparing odometry ({self.odom_frame_}->{self.base_frame_}, '
            f'topic {self.odom_topic_}) with slam_toolbox '
            f'({self.map_frame_}->{self.base_frame_}, topic '
            f'{self.slam_pose_topic_})')
        self.get_logger().info(f'Logging to {self.csv_path_}')

    # ------------------------------------------------------------------ input

    def odom_callback(self, msg):
        self.odom_msg_ = msg
        self.odom_count_ += 1
        now = time.time()
        if self.odom_first_t_ is None:
            self.odom_first_t_ = now
        self.odom_last_t_ = now

    def pose_callback(self, msg):
        self.pose_msg_ = msg
        self.pose_count_ += 1
        now = time.time()
        if self.pose_first_t_ is None:
            self.pose_first_t_ = now
        else:
            self.pose_intervals_.append(now - self.pose_last_t_)
        self.pose_last_t_ = now

        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.pose_prev_xy_ is not None:
            self.pose_steps_.append(math.hypot(
                xy[0] - self.pose_prev_xy_[0], xy[1] - self.pose_prev_xy_[1]))
        self.pose_prev_xy_ = xy

    # ----------------------------------------------------------------- helper

    def lookup(self, parent, child):
        """Return (x, y, yaw) of child in parent, or None if unavailable."""
        try:
            tf = self.tf_buffer_.lookup_transform(parent, child, Time())
        except tf2_ros.TransformException as ex:
            self.tf_failures_ += 1
            self.get_logger().warn(
                f'TF {parent} -> {child} unavailable: {ex}',
                throttle_duration_sec=5.0)
            return None
        return (tf.transform.translation.x,
                tf.transform.translation.y,
                yaw_from_quaternion(tf.transform.rotation))

    # ----------------------------------------------------------------- sample

    def sample(self):
        elapsed = time.time() - self.start_wall_
        if self.duration_ > 0.0 and elapsed >= self.duration_:
            self.print_summary()
            raise SystemExit(0)

        odom = self.lookup(self.odom_frame_, self.base_frame_)
        slam = self.lookup(self.map_frame_, self.base_frame_)
        corr = self.lookup(self.map_frame_, self.odom_frame_)
        if odom is None or slam is None or corr is None:
            return

        # Sanity check: the /odom topic and the odom->base_link TF should agree.
        if self.odom_msg_ is not None:
            gap = math.hypot(
                self.odom_msg_.pose.pose.position.x - odom[0],
                self.odom_msg_.pose.pose.position.y - odom[1])
            self.max_odom_topic_tf_gap_ = max(self.max_odom_topic_tf_gap_, gap)

        diff_x = slam[0] - odom[0]
        diff_y = slam[1] - odom[1]
        diff_dist = math.hypot(diff_x, diff_y)
        diff_yaw = normalize_angle(slam[2] - odom[2])

        if self.prev_odom_xy_ is not None:
            self.odom_path_len_ += math.hypot(
                odom[0] - self.prev_odom_xy_[0], odom[1] - self.prev_odom_xy_[1])
        if self.prev_slam_xy_ is not None:
            self.slam_path_len_ += math.hypot(
                slam[0] - self.prev_slam_xy_[0], slam[1] - self.prev_slam_xy_[1])
        self.prev_odom_xy_ = (odom[0], odom[1])
        self.prev_slam_xy_ = (slam[0], slam[1])

        self.samples_ += 1
        self.sum_dist_ += diff_dist
        self.sum_yaw_ += abs(diff_yaw)
        self.max_dist_ = max(self.max_dist_, diff_dist)
        self.max_yaw_ = max(self.max_yaw_, abs(diff_yaw))

        if self.pose_msg_ is not None:
            p = self.pose_msg_.pose.pose
            pose_stamp = Time.from_msg(self.pose_msg_.header.stamp)
            pose_age = (self.get_clock().now() - pose_stamp).nanoseconds * 1e-9
            pose_cols = [round(p.position.x, 4), round(p.position.y, 4),
                         round(math.degrees(yaw_from_quaternion(p.orientation)), 3),
                         round(pose_age, 3)]
        else:
            pose_cols = ['', '', '', '']

        row = [round(elapsed, 3),
               round(odom[0], 4), round(odom[1], 4), round(math.degrees(odom[2]), 3),
               round(slam[0], 4), round(slam[1], 4), round(math.degrees(slam[2]), 3),
               round(diff_x, 4), round(diff_y, 4), round(diff_dist, 4),
               round(math.degrees(diff_yaw), 3),
               round(corr[0], 4), round(corr[1], 4), round(math.degrees(corr[2]), 3),
               round(self.odom_path_len_, 3), round(self.slam_path_len_, 3)] + pose_cols
        self.csv_writer_.writerow(row)
        self.last_row_ = row

        if elapsed - self.last_print_ >= self.print_period_:
            self.last_print_ = elapsed
            self.get_logger().info(
                f'[{elapsed:6.1f}s] '
                f'odom ({odom[0]:7.3f}, {odom[1]:7.3f}, {math.degrees(odom[2]):7.2f}deg) | '
                f'slam ({slam[0]:7.3f}, {slam[1]:7.3f}, {math.degrees(slam[2]):7.2f}deg) | '
                f'diff {diff_dist:6.3f} m / {math.degrees(diff_yaw):6.2f} deg | '
                f'/pose msgs {self.pose_count_}')

    # ---------------------------------------------------------------- summary

    def print_summary(self):
        if self.summary_printed_:
            return
        self.summary_printed_ = True
        elapsed = time.time() - self.start_wall_
        lines = ['', '=' * 72, 'odom vs slam_toolbox summary', '=' * 72,
                 f'duration                : {elapsed:.1f} s',
                 f'samples                 : {self.samples_} '
                 f'(TF lookup failures: {self.tf_failures_})']

        if self.odom_count_ >= 2 and self.odom_last_t_ > self.odom_first_t_:
            rate = (self.odom_count_ - 1) / (self.odom_last_t_ - self.odom_first_t_)
            lines.append(f'{self.odom_topic_:<24}: {self.odom_count_} msgs '
                         f'({rate:.1f} Hz)')
        else:
            lines.append(f'{self.odom_topic_:<24}: {self.odom_count_} msgs')

        if self.pose_count_ >= 2 and self.pose_last_t_ > self.pose_first_t_:
            rate = (self.pose_count_ - 1) / (self.pose_last_t_ - self.pose_first_t_)
            mean_gap = sum(self.pose_intervals_) / len(self.pose_intervals_)
            lines.append(f'{self.slam_pose_topic_:<24}: {self.pose_count_} msgs '
                         f'({rate:.2f} Hz, mean gap {mean_gap:.2f} s, '
                         f'max gap {max(self.pose_intervals_):.2f} s)')
            if self.pose_steps_:
                mean_step = sum(self.pose_steps_) / len(self.pose_steps_)
                lines.append(f'{"":<24}  mean travel between /pose msgs: '
                             f'{mean_step:.3f} m '
                             f'(~= minimum_travel_distance)')
        else:
            lines.append(f'{self.slam_pose_topic_:<24}: {self.pose_count_} msgs '
                         f'(event driven - it only fires when slam_toolbox '
                         f'accepts a new scan)')

        if self.samples_ > 0:
            lines += [
                '-' * 72,
                f'path length (odom)      : {self.odom_path_len_:.3f} m',
                f'path length (slam)      : {self.slam_path_len_:.3f} m',
                f'mean |slam - odom|      : {self.sum_dist_ / self.samples_:.3f} m'
                f' / {math.degrees(self.sum_yaw_ / self.samples_):.2f} deg',
                f'max  |slam - odom|      : {self.max_dist_:.3f} m'
                f' / {math.degrees(self.max_yaw_):.2f} deg',
            ]
            if self.last_row_ is not None:
                lines.append(
                    f'final |slam - odom|     : {self.last_row_[9]:.3f} m'
                    f' / {self.last_row_[10]:.2f} deg')
                lines.append(
                    f'final map->odom corr.   : '
                    f'({self.last_row_[11]:.3f}, {self.last_row_[12]:.3f}) m, '
                    f'{self.last_row_[13]:.2f} deg')
            if self.odom_path_len_ > 0.1:
                lines.append(
                    f'drift / distance driven : '
                    f'{100.0 * self.max_dist_ / self.odom_path_len_:.1f} % '
                    f'(max correction over {self.odom_path_len_:.2f} m)')
            lines.append(
                f'max |/odom topic - TF|  : {self.max_odom_topic_tf_gap_:.4f} m '
                f'(should stay ~0; a large value means something other than the '
                f'swerve controller owns {self.odom_frame_}->{self.base_frame_})')

        lines += ['-' * 72, f'CSV: {self.csv_path_}', '=' * 72, '']
        self.get_logger().info('\n'.join(lines))

    def destroy_node(self):
        try:
            self.csv_file_.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomSlamTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.print_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
