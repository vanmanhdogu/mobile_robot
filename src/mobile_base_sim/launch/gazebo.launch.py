# Copyright 2026 Dogu
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

"""Ignition Gazebo simulation of the AntBot mobile base (body, wheels, 2D LiDAR)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def _resolve_world(context, *args, **kwargs):
    """Resolve a named world or a direct SDF path, then start the server."""
    world_value = LaunchConfiguration('world').perform(context)
    pkg = get_package_share_directory('mobile_base_sim')

    if os.path.isfile(world_value):
        world_sdf = world_value
    else:
        worlds_dir = os.path.join(pkg, 'worlds')
        worlds_yaml = os.path.join(pkg, 'config', 'worlds.yaml')
        world_sdf = os.path.join(worlds_dir, world_value + '.sdf')

        if os.path.isfile(worlds_yaml):
            with open(worlds_yaml, 'r') as f:
                worlds = (yaml.safe_load(f) or {}).get('worlds', {})
            if world_value in worlds:
                world_sdf = os.path.join(worlds_dir, worlds[world_value]['sdf'])

        if not os.path.isfile(world_sdf):
            raise FileNotFoundError(
                f"World '{world_value}' not found. Looked for: {world_sdf}")

    # '-r' starts the physics clock immediately, '-s' drops the GUI.
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    cmd = ['ign', 'gazebo', '-r'] + (['-s'] if headless else []) + [world_sdf]

    return [ExecuteProcess(cmd=cmd, output='screen')]


def _robot(context, *args, **kwargs):
    """Spawn the robot, publish its description, start controllers and bridge.

    xacro runs here rather than at module scope so that the spawn pose, which
    is a LaunchConfiguration, can be resolved.
    """
    pkg = get_package_share_directory('mobile_base_sim')

    urdf_path = os.path.join(pkg, 'urdf', 'mobile_base.xacro')
    robot_description_xml = xacro.process_file(urdf_path).toxml()

    spawn_x = LaunchConfiguration('x').perform(context)
    spawn_y = LaunchConfiguration('y').perform(context)
    spawn_z = LaunchConfiguration('z').perform(context)
    spawn_yaw = LaunchConfiguration('yaw').perform(context)

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mobile_base',
            '-string', robot_description_xml,
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw,
        ],
        output='screen')

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_xml},
            {'use_sim_time': True},
        ])

    controller_yaml = os.path.join(pkg, 'config', 'swerve_controller.yaml')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file', controller_yaml,
            '--controller-manager-timeout', '30',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    swerve_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'swerve_controller',
            '--param-file', controller_yaml,
            '--controller-manager-timeout', '30',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    # Controllers must be spawned in order and only once the robot exists:
    # the controller_manager lives inside the Gazebo plugin, so it does not
    # answer any service until the model has been created.
    jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot,
                      on_exit=[joint_state_broadcaster_spawner]))
    swerve_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster_spawner,
                      on_exit=[swerve_controller_spawner]))

    # Only the LaserScan half of each gpu_lidar is bridged; the point cloud on
    # <topic>/points is left on the Gazebo side.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan_0@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan_1@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    actions = [spawn_robot, robot_state_pub, jsb_after_spawn,
               swerve_after_jsb, gz_bridge]

    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'mobile_base.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'))

    return actions


def generate_launch_description():
    description_pkg = get_package_share_directory('antbot_description')

    ros_distro = os.environ.get('ROS_DISTRO', 'humble')

    # antbot_description's meshes are referenced as package://antbot_description/...
    # Ignition resolves that by searching IGN_GAZEBO_RESOURCE_PATH for a
    # directory named antbot_description, so the path to add is its parent.
    resource_path = os.path.dirname(description_pkg)
    existing_resource = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    set_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        resource_path + (':' + existing_resource if existing_resource else ''))

    # Appended, not overwritten: on arm64 libign_ros2_control-system.so is
    # built into the workspace overlay and docker-compose.yml already points
    # this variable there.
    plugin_path = os.path.join('/opt', 'ros', ros_distro, 'lib')
    existing_plugin = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    set_plugin_path = SetEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        plugin_path + (':' + existing_plugin if existing_plugin else ''))

    args = [
        DeclareLaunchArgument(
            'world', default_value='box_room',
            description='World name from config/worlds.yaml, or a full SDF path'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the Gazebo server without the GUI (ign gazebo -s)'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Also start RViz with rviz/mobile_base.rviz'),
        DeclareLaunchArgument('x', default_value='0.0',
                              description='Spawn position x [m]'),
        DeclareLaunchArgument('y', default_value='0.0',
                              description='Spawn position y [m]'),
        DeclareLaunchArgument('z', default_value='0.15',
                              description='Spawn position z [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description='Spawn yaw [rad]'),
    ]

    return LaunchDescription(args + [
        set_resource_path,
        set_plugin_path,
        OpaqueFunction(function=_resolve_world),
        OpaqueFunction(function=_robot),
    ])
