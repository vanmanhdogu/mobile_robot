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
# Author: Yeeun Hwang

"""Spawn semi_humanoid_v2 (ANTBot + OpenArm v2.0 on a lift) in Ignition Gazebo Fortress.

Adapted from antbot_gazebo/launch/gazebo.launch.py: same worlds, same swerve
controller and sensor bridges, plus the two arm and gripper controllers.

model picks the description: semi_humanoid_v2 (default) or semi_humanoid_v3.

  Sensor                 Argument (default)          ROS topic
  2D LiDAR front/back    always on                   /scan_0, /scan_1
  IMU                    always on                   /imu/data
  S10 front              camera (true)               /sensor/camera/stereo_front/*
  S10 back/left/right    side_cameras (true)         /sensor/camera/stereo_<pos>/*
  Airy, front            airy (false) *              /airy_points
  Airy, rear             airy_back (false) *         /airy_back_points
                         airy_vertical_samples (96)  192 / 96 / 48 lines

* model:=semi_humanoid_v3 only — semi_humanoid_v2.urdf.xacro has no 3D LiDAR.
  Both are off by default, so a bare launch costs what it always did. The old
  stock 3D LiDAR mount has been removed from every model in this package.
"""

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


def _resolve_world_path(context, *args, **kwargs):
    """Resolve a named world or direct SDF path, then launch Gazebo."""
    world_value = LaunchConfiguration('world').perform(context)

    if os.path.isfile(world_value):
        world_sdf = world_value
    else:
        gazebo_pkg = get_package_share_directory('antbot_gazebo')
        worlds_yaml = os.path.join(gazebo_pkg, 'config', 'worlds.yaml')
        worlds_dir = os.path.join(gazebo_pkg, 'worlds')

        if os.path.isfile(worlds_yaml):
            with open(worlds_yaml, 'r') as f:
                config = yaml.safe_load(f)
            worlds = config.get('worlds', {})
            if world_value in worlds:
                world_sdf = os.path.join(worlds_dir, worlds[world_value]['sdf'])
            else:
                world_sdf = os.path.join(worlds_dir, world_value + '.sdf')
        else:
            world_sdf = os.path.join(worlds_dir, world_value + '.sdf')

        if not os.path.isfile(world_sdf):
            raise FileNotFoundError(
                f"World '{world_value}' not found. Looked for: {world_sdf}")

    # '-s' runs the server without the GUI: useful on a Jetson, over SSH, and
    # for CI. Sensors still publish normally in this mode.
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    cmd = ['ign', 'gazebo', '-r'] + (['-s'] if headless else []) + [world_sdf]

    return [ExecuteProcess(cmd=cmd, output='screen')]


def _robot_and_bridge(context, *args, **kwargs):
    """
    Build the URDF and the gz<->ROS bridge from the resolved sensor flags.

    xacro has to run here rather than in generate_launch_description() because
    the sensor toggles are LaunchConfigurations and are only resolvable once a
    launch context exists.
    """
    pkg = get_package_share_directory('semi_humanoid_v2')

    camera = LaunchConfiguration('camera').perform(context).lower()
    side_cameras = LaunchConfiguration('side_cameras').perform(context).lower()
    calibration_yaml_path = LaunchConfiguration(
        'calibration_yaml_path').perform(context)

    model = LaunchConfiguration('model').perform(context)

    mappings = {
        'camera': camera,
        'side_cameras': side_cameras,
        'calibration_yaml_path': calibration_yaml_path,
    }

    # The Airys are declared only by semi_humanoid_v3.urdf.xacro. xacro would
    # ignore the mappings on any other model, but the bridges below would
    # still come up with nothing publishing to them, so pin them off.
    airy = airy_back = 'false'
    if model == 'semi_humanoid_v3':
        airy = LaunchConfiguration('airy').perform(context).lower()
        airy_back = LaunchConfiguration('airy_back').perform(context).lower()
        mappings.update({
            'airy': airy,
            'airy_back': airy_back,
            'airy_vertical_samples':
                LaunchConfiguration('airy_vertical_samples').perform(context),
        })

    urdf_path = os.path.join(pkg, 'urdf', model + '.urdf.xacro')
    robot_description_xml = xacro.process_file(
        urdf_path, mappings=mappings).toxml()

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', model,
            '-string', robot_description_xml,
            '-x', '0.0', '-y', '0.0', '-z', '0.15',
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

    controller_yaml = os.path.join(pkg, 'config', 'controllers_gazebo.yaml')

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
            'antbot_swerve_controller',
            '--param-file', controller_yaml,
            '--controller-manager-timeout', '30',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    arm_controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'lift_controller',
            'left_arm_controller', 'right_arm_controller',
            'left_gripper_controller', 'right_gripper_controller',
            '--param-file', controller_yaml,
            '--controller-manager-timeout', '30',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    joint_state_broadcaster_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner]))

    swerve_controller_after_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_controller_spawner, arm_controllers_spawner]))

    # ── gz -> ROS bridge topics ──
    bridge_args = [
        '/scan_0@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/scan_1@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    ]
    bridge_remaps = [
        ('/imu', '/imu/data'),
    ]

    def _bridge_rgbd(position):
        """
        Bridge the four topics an rgbd_camera publishes for one S10.

        Remapped onto /sensor/camera/stereo_<position>/*, which is what the
        real S10 drivers publish, so perception nodes need no sim-specific
        config. The gz-side name follows the link (camera_s10_<position>);
        only the ROS-facing name is the driver contract.
        """
        gz_topic = '/camera_s10_' + position
        ros_topic = '/sensor/camera/stereo_' + position
        bridge_args.extend([
            gz_topic + '/image@sensor_msgs/msg/Image'
            '[ignition.msgs.Image',
            gz_topic + '/depth_image@sensor_msgs/msg/Image'
            '[ignition.msgs.Image',
            gz_topic + '/points@sensor_msgs/msg/PointCloud2'
            '[ignition.msgs.PointCloudPacked',
            gz_topic + '/camera_info@sensor_msgs/msg/CameraInfo'
            '[ignition.msgs.CameraInfo',
        ])
        bridge_remaps.extend([
            (gz_topic + '/image', ros_topic + '/image_raw'),
            (gz_topic + '/depth_image', ros_topic + '/depth/image_raw'),
            (gz_topic + '/points', ros_topic + '/points'),
            (gz_topic + '/camera_info', ros_topic + '/camera_info'),
        ])

    # Only the point clouds for the Airys: one ring of a 3D scan is not
    # useful as a LaserScan.
    for gz_name, ros_name, enabled in (
            ('airy', 'airy_points', airy),
            ('airy_back', 'airy_back_points', airy_back)):
        if enabled != 'true':
            continue
        bridge_args.append(
            '/' + gz_name + '/points@sensor_msgs/msg/PointCloud2'
            '[ignition.msgs.PointCloudPacked')
        bridge_remaps.append(('/' + gz_name + '/points', '/' + ros_name))

    if camera == 'true':
        _bridge_rgbd('front')

    if side_cameras == 'true':
        for position in ('back', 'left', 'right'):
            _bridge_rgbd(position)

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=bridge_remaps,
        parameters=[{'use_sim_time': True}],
        output='screen')

    return [
        spawn_robot,
        robot_state_pub,
        joint_state_broadcaster_after_spawn,
        swerve_controller_after_joint_state_broadcaster,
        gz_bridge,
    ]


def generate_launch_description():
    description_pkg = get_package_share_directory('antbot_description')
    openarm_pkg = get_package_share_directory('semi_humanoid_v2')

    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    plugin_path = os.path.join('/opt', 'ros', ros_distro, 'lib')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World name (resolved via config/worlds.yaml) or full path to SDF file')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run the Gazebo server without the GUI (ign gazebo -s)')

    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='true',
        description='Enable the front S10 RGBD camera '
                    '(-> /sensor/camera/stereo_front/*)')

    side_cameras_arg = DeclareLaunchArgument(
        'side_cameras',
        default_value='true',
        description='Enable the back/left/right S10 RGBD cameras '
                    '(-> /sensor/camera/stereo_<position>/*)')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='semi_humanoid_v2',
        choices=['semi_humanoid_v2', 'semi_humanoid_v3'],
        description='Robot description: urdf/<model>.urdf.xacro')

    # The three below apply to model:=semi_humanoid_v3 only; they are ignored,
    # and no bridge is started, for semi_humanoid_v2.
    airy_arg = DeclareLaunchArgument(
        'airy',
        default_value='false',
        description='v3 only. RoboSense Airy 3D LiDAR, front mount '
                    '(-> /airy_points)')

    airy_back_arg = DeclareLaunchArgument(
        'airy_back',
        default_value='false',
        description='v3 only. RoboSense Airy 3D LiDAR, rear mount '
                    '(-> /airy_back_points). With the front unit it leaves a '
                    '0.554 m blind slab across the mid-body - see SENSORS.md.')

    airy_vertical_samples_arg = DeclareLaunchArgument(
        'airy_vertical_samples',
        default_value='96',
        choices=['192', '96', '48'],
        description='v3 only. Airy beam count, both units. 96 is the datasheet '
                    '96-beam mode; drop to 48 if the sim falls behind.')

    calibration_yaml_path_arg = DeclareLaunchArgument(
        'calibration_yaml_path',
        default_value='',
        description='Optional sensor extrinsic yaml, same file the real '
                    'robot loads. Empty uses the nominal mounts.')

    # Meshes are package:// URIs into two packages; with an isolated colcon
    # install they live under different share/ dirs, so expose both.
    resource_path = ':'.join(sorted({
        os.path.dirname(description_pkg), os.path.dirname(openarm_pkg)}))
    existing_resource = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    set_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        resource_path + (':' + existing_resource if existing_resource else ''))

    existing_plugin = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    set_plugin_path = SetEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        plugin_path + (':' + existing_plugin if existing_plugin else ''))

    ign_gazebo = OpaqueFunction(function=_resolve_world_path)
    robot_and_bridge = OpaqueFunction(function=_robot_and_bridge)

    return LaunchDescription([
        world_arg,
        headless_arg,
        camera_arg,
        side_cameras_arg,
        model_arg,
        airy_arg,
        airy_back_arg,
        airy_vertical_samples_arg,
        calibration_yaml_path_arg,
        set_resource_path,
        set_plugin_path,
        ign_gazebo,
        robot_and_bridge,
    ])
