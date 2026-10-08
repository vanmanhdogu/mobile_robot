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
Plot a CSV recorded by ``odom_slam_test.py``.

Renders six panels: the two trajectories, the position gap between them, the
two headings, the ``map->odom`` correction, the cumulative path lengths and the
age of the last ``/pose`` message (which shows how event driven that topic is).

Usage::

    ./odom_slam_plot.py odom_slam.csv                 # writes odom_slam.png
    ./odom_slam_plot.py odom_slam.csv -o /tmp/out.png
"""

import argparse
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt   # noqa: E402
import numpy as np                # noqa: E402

# Categorical slots 1-3 of the validated default palette (light surface).
C_ODOM = '#2a78d6'
C_SLAM = '#eb6834'
C_THIRD = '#1baf7a'
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_2 = '#52514e'
GRID = '#dedcd5'

LW = 1.8


def load(path):
    """Read the CSV into a dict of float arrays (empty cells become NaN)."""
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f'{path} contains no samples')
    data = {}
    for key in rows[0]:
        col = []
        for r in rows:
            v = r[key]
            col.append(float(v) if v not in ('', None) else math.nan)
        data[key] = np.array(col)
    return data


def style(ax, title, xlabel, ylabel):
    ax.set_title(title, color=INK, fontsize=11, loc='left', pad=8)
    ax.set_xlabel(xlabel, color=INK_2, fontsize=9)
    ax.set_ylabel(ylabel, color=INK_2, fontsize=9)
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.7)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK_2, labelsize=8.5)


def legend(ax, **kwargs):
    kwargs.setdefault('loc', 'best')
    leg = ax.legend(frameon=True, fontsize=8.5, framealpha=0.9, **kwargs)
    leg.get_frame().set_facecolor(SURFACE)
    leg.get_frame().set_edgecolor(GRID)
    for text in leg.get_texts():
        text.set_color(INK_2)


def pose_update_times(t, age):
    """Times at which /pose delivered a new message (its age drops back to ~0)."""
    out = []
    prev = math.nan
    for ti, ai in zip(t, age):
        if not math.isnan(ai) and (math.isnan(prev) or ai < prev - 0.05):
            out.append(ti)
        prev = ai
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv', help='CSV written by odom_slam_test.py')
    p.add_argument('-o', '--output', default=None, help='output PNG path')
    args = p.parse_args()

    d = load(args.csv)
    out = args.output or os.path.splitext(args.csv)[0] + '.png'

    t = d['t']
    updates = pose_update_times(t, d['pose_topic_age'])
    odom_yaw = np.degrees(np.unwrap(np.radians(d['odom_yaw_deg'])))
    slam_yaw = np.degrees(np.unwrap(np.radians(d['slam_yaw_deg'])))

    fig, axes = plt.subplots(3, 2, figsize=(14, 13))
    fig.patch.set_facecolor(SURFACE)

    duration = t[-1] - t[0]
    n_pose = int(np.sum(~np.isnan(d['pose_topic_age'])) > 0) and len(updates)
    fig.suptitle(
        'Odometry vs slam_toolbox  -  '
        f'{duration:.0f} s, {d["odom_path_len"][-1]:.1f} m driven, '
        f'max gap {np.nanmax(d["diff_dist"]):.3f} m / '
        f'{np.nanmax(np.abs(d["diff_yaw_deg"])):.1f} deg, '
        f'{n_pose} /pose updates',
        color=INK, fontsize=13, x=0.012, ha='left', y=0.985)

    # 1. Trajectories -------------------------------------------------------
    ax = axes[0][0]
    ax.plot(d['odom_x'], d['odom_y'], color=C_ODOM, linewidth=LW,
            label='odom -> base_link')
    ax.plot(d['slam_x'], d['slam_y'], color=C_SLAM, linewidth=LW,
            label='map -> base_link (SLAM)')
    ax.plot(d['odom_x'][0], d['odom_y'][0], 'o', color=INK_2, markersize=8,
            markeredgecolor=SURFACE, markeredgewidth=2, label='start')
    ax.plot(d['odom_x'][-1], d['odom_y'][-1], 's', color=C_ODOM, markersize=8,
            markeredgecolor=SURFACE, markeredgewidth=2)
    ax.plot(d['slam_x'][-1], d['slam_y'][-1], 's', color=C_SLAM, markersize=8,
            markeredgecolor=SURFACE, markeredgewidth=2)
    xs = np.concatenate([d['odom_x'], d['slam_x']])
    ys = np.concatenate([d['odom_y'], d['slam_y']])
    pad = 0.08 * max(xs.ptp(), ys.ptp(), 1.0)
    ax.set_xlim(xs.min() - pad, xs.max() + pad)
    ax.set_ylim(ys.min() - pad, ys.max() + pad)
    ax.set_aspect('equal', adjustable='box')
    style(ax, 'Trajectory (squares = end pose)', 'x [m]', 'y [m]')
    legend(ax, loc='upper left', bbox_to_anchor=(1.01, 1.0))

    # 2. Position gap -------------------------------------------------------
    ax = axes[0][1]
    ax.plot(t, d['diff_dist'], color=C_SLAM, linewidth=LW,
            label='|SLAM - odom|')
    ax.fill_between(t, 0, d['diff_dist'], color=C_SLAM, alpha=0.12)
    top = float(np.nanmax(d['diff_dist'])) or 1.0
    ax.set_ylim(-0.06 * top, 1.12 * top)
    ax.plot(updates, [-0.03 * top] * len(updates), '|', color=C_THIRD,
            markersize=7, markeredgewidth=1.4,
            label=f'/pose update ({len(updates)}x)')
    style(ax, 'Position gap = accumulated odometry drift SLAM removed',
          't [s]', 'distance [m]')
    legend(ax)

    # 3. Heading ------------------------------------------------------------
    ax = axes[1][0]
    ax.plot(t, odom_yaw, color=C_ODOM, linewidth=LW, label='odom yaw')
    ax.plot(t, slam_yaw, color=C_SLAM, linewidth=LW, label='SLAM yaw')
    style(ax, 'Heading (unwrapped)', 't [s]', 'yaw [deg]')
    legend(ax)

    # 4. Heading gap --------------------------------------------------------
    ax = axes[1][1]
    ax.axhline(0.0, color=GRID, linewidth=1.0)
    ax.plot(t, d['diff_yaw_deg'], color=C_SLAM, linewidth=LW,
            label='SLAM yaw - odom yaw')
    ax.fill_between(t, 0, d['diff_yaw_deg'], color=C_SLAM, alpha=0.12)
    style(ax, 'Heading gap (same correction, rotational part)',
          't [s]', 'yaw difference [deg]')
    legend(ax)

    # 5. Cumulative path length --------------------------------------------
    ax = axes[2][0]
    ax.plot(t, d['odom_path_len'], color=C_ODOM, linewidth=LW,
            label='odom path')
    ax.plot(t, d['slam_path_len'], color=C_SLAM, linewidth=LW,
            label='SLAM path')
    ax.annotate(f'{d["odom_path_len"][-1]:.2f} m',
                (t[-1], d['odom_path_len'][-1]), textcoords='offset points',
                xytext=(-6, -14), ha='right', color=C_ODOM, fontsize=9)
    ax.annotate(f'{d["slam_path_len"][-1]:.2f} m',
                (t[-1], d['slam_path_len'][-1]), textcoords='offset points',
                xytext=(-6, 8), ha='right', color=C_SLAM, fontsize=9)
    style(ax, 'Distance travelled', 't [s]', 'path length [m]')
    legend(ax)

    # 6. /pose age ----------------------------------------------------------
    ax = axes[2][1]
    ax.plot(t, d['pose_topic_age'], color=C_THIRD, linewidth=LW,
            label='age of last /pose message')
    style(ax, '/pose is event driven: age grows until a scan is accepted',
          't [s]', 'age [s]')
    legend(ax)

    fig.tight_layout(rect=(0, 0, 1, 0.965))
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
