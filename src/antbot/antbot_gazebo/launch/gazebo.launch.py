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
    """Build the URDF and the gz<->ROS bridge from the resolved sensor flags.

    xacro has to run here rather than in generate_launch_description() because
    the sensor toggles are LaunchConfigurations and are only resolvable once a
    launch context exists.
    """
    gazebo_pkg = get_package_share_directory('antbot_gazebo')

    lidar_3d = LaunchConfiguration('lidar_3d').perform(context).lower()
    camera = LaunchConfiguration('camera').perform(context).lower()
    side_cameras = LaunchConfiguration('side_cameras').perform(context).lower()
    mono_cameras = LaunchConfiguration('mono_cameras').perform(context).lower()

    urdf_path = os.path.join(gazebo_pkg, 'urdf', 'antbot_sim.xacro')
    robot_description_xml = xacro.process_file(
        urdf_path,
        mappings={
            'lidar_3d': lidar_3d,
            'camera': camera,
            'side_cameras': side_cameras,
            'mono_cameras': mono_cameras,
        }).toxml()

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'antbot',
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

    controller_yaml = os.path.join(
        gazebo_pkg, 'config', 'swerve_controller_gazebo.yaml')

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

    joint_state_broadcaster_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner]))

    swerve_controller_after_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_controller_spawner]))

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

    if lidar_3d == 'true':
        # gpu_lidar publishes the point cloud on <topic>/points. The flat
        # LaserScan on /lidar_3d is left unbridged: a single ring of a 3D
        # scan is not useful and Nav2 already gets /scan_0 and /scan_1.
        bridge_args.append(
            '/lidar_3d/points@sensor_msgs/msg/PointCloud2'
            '[ignition.msgs.PointCloudPacked')
        # Match the real Vanjee WLR-722 driver's topic name.
        bridge_remaps.append(('/lidar_3d/points', '/lidar_3d_points'))

    def _bridge_rgbd(position):
        """Bridge the four topics an rgbd_camera publishes for one position.

        Remapped onto /sensor/camera/stereo_<position>/*, the names the real
        S10 drivers use, so perception nodes need no sim-specific config.
        """
        gz_topic = '/camera_stereo_' + position
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

    if camera == 'true':
        _bridge_rgbd('front')

    if side_cameras == 'true':
        for position in ('left', 'right', 'back'):
            _bridge_rgbd(position)

    if mono_cameras == 'true':
        # Remapped onto the same names antbot_camera uses on real hardware,
        # so perception nodes need no sim-specific topic config.
        for position in ('front', 'left', 'right', 'back'):
            gz_topic = '/camera_mono_' + position
            ros_topic = '/sensor/camera/v4l2_driver/' + position
            bridge_args += [
                gz_topic + '/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                gz_topic + '/camera_info@sensor_msgs/msg/CameraInfo'
                '[ignition.msgs.CameraInfo',
            ]
            bridge_remaps += [
                (gz_topic + '/image', ros_topic + '/image_raw'),
                (gz_topic + '/camera_info', ros_topic + '/camera_info'),
            ]

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

    lidar_3d_arg = DeclareLaunchArgument(
        'lidar_3d',
        default_value='false',
        description='Enable the simulated 3D LiDAR (gpu_lidar -> /lidar_3d_points)')

    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='false',
        description='Enable the front RGBD camera '
                    '(-> /sensor/camera/stereo_front/*)')

    side_cameras_arg = DeclareLaunchArgument(
        'side_cameras',
        default_value='false',
        description='Enable the left/right/back RGBD cameras '
                    '(-> /sensor/camera/stereo_<position>/*)')

    mono_cameras_arg = DeclareLaunchArgument(
        'mono_cameras',
        default_value='false',
        description='Enable the 4 mono cameras '
                    '(-> /sensor/camera/v4l2_driver/<position>/*). Expensive.')

    resource_path = os.path.dirname(description_pkg)
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
        lidar_3d_arg,
        camera_arg,
        side_cameras_arg,
        mono_cameras_arg,
        set_resource_path,
        set_plugin_path,
        ign_gazebo,
        robot_and_bridge,
    ])
