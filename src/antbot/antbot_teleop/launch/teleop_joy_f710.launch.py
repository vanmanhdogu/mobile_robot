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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('antbot_teleop'),
        'config', 'joy_f710.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML with joy_node and teleop_joystick_f710 parameters')
    params_file = LaunchConfiguration('params_file')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[params_file])

    teleop_node = Node(
        package='antbot_teleop',
        executable='teleop_joystick_f710',
        name='teleop_joystick_f710',
        output='screen',
        parameters=[params_file])

    ld = LaunchDescription()
    ld.add_action(params_arg)
    ld.add_action(joy_node)
    ld.add_action(teleop_node)

    return ld
