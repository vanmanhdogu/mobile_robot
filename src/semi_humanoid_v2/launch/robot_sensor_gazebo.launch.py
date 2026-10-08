"""Real-robot model of semi_humanoid_v2 in Ignition Gazebo, with simulated sensors.

Spawns semi_humanoid_v2_real.urdf.xacro (base wired like antbot_description:
real, upside-down 2D LiDAR mounts and calibration support) with sim_gazebo:=true,
and bridges the same sensor set as antbot_gazebo/launch/gazebo.launch.py:

  sensor            arg (default)          ROS topic
  2D LiDAR front    always                 /scan_0
  2D LiDAR back     always                 /scan_1
  IMU               always                 /imu/data
  clock             always                 /clock
  3D LiDAR          lidar_3d (false)       /lidar_3d_points
  S10 front         camera (false)         /sensor/camera/stereo_front/*
  S10 back/left/right  side_cameras (false)  /sensor/camera/stereo_<position>/*

Controllers: joint_state_broadcaster, antbot_swerve_controller, lift_controller and
the arm and gripper controllers, all from config/controllers_gazebo.yaml.
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
    """Resolve a named world (antbot_gazebo/config/worlds.yaml) or an SDF path."""
    world_value = LaunchConfiguration('world').perform(context)

    if os.path.isfile(world_value):
        world_sdf = world_value
    else:
        gazebo_pkg = get_package_share_directory('antbot_gazebo')
        worlds_yaml = os.path.join(gazebo_pkg, 'config', 'worlds.yaml')
        worlds_dir = os.path.join(gazebo_pkg, 'worlds')
        worlds = {}
        if os.path.isfile(worlds_yaml):
            with open(worlds_yaml, 'r') as f:
                worlds = yaml.safe_load(f).get('worlds', {})
        sdf = worlds[world_value]['sdf'] if world_value in worlds else world_value + '.sdf'
        world_sdf = os.path.join(worlds_dir, sdf)
        if not os.path.isfile(world_sdf):
            raise FileNotFoundError(
                f"World '{world_value}' not found. Looked for: {world_sdf}")

    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    cmd = ['ign', 'gazebo', '-r'] + (['-s'] if headless else []) + [world_sdf]
    return [ExecuteProcess(cmd=cmd, output='screen')]


def _robot_and_bridge(context, *args, **kwargs):
    """Build the URDF from the sensor flags, spawn it, and bridge its sensors."""
    pkg = get_package_share_directory('semi_humanoid_v2')

    flags = {name: LaunchConfiguration(name).perform(context).lower()
             for name in ('lidar_3d', 'camera', 'side_cameras')}
    calibration_yaml_path = os.path.expanduser(
        LaunchConfiguration('calibration_yaml_path').perform(context))

    robot_description_xml = xacro.process_file(
        os.path.join(pkg, 'urdf', 'semi_humanoid_v2_real.urdf.xacro'),
        mappings={
            'sim_gazebo': 'true',
            'calibration_yaml_path': calibration_yaml_path,
            **flags,
        }).toxml()

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'semi_humanoid_v2',
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

    def spawner(*controllers):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[*controllers, '--param-file', controller_yaml,
                       '--controller-manager-timeout', '30'],
            parameters=[{'use_sim_time': True}],
            output='screen')

    joint_state_broadcaster_spawner = spawner('joint_state_broadcaster')

    # ── gz -> ROS bridge (same table as antbot_gazebo) ──
    bridge_args = [
        '/scan_0@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/scan_1@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    ]
    bridge_remaps = [('/imu', '/imu/data')]

    if flags['lidar_3d'] == 'true':
        # Only the point cloud: one ring of a 3D scan is not useful as a
        # LaserScan. Topic name matches the real Vanjee WLR-722 driver.
        bridge_args.append(
            '/lidar_3d/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked')
        bridge_remaps.append(('/lidar_3d/points', '/lidar_3d_points'))

    def bridge_rgbd(position):
        """Bridge one S10 onto /sensor/camera/stereo_<position>/*, the real driver names."""
        gz_topic = '/camera_s10_' + position
        ros_topic = '/sensor/camera/stereo_' + position
        for suffix, msg, ros_suffix in (
                ('image', 'sensor_msgs/msg/Image[ignition.msgs.Image', 'image_raw'),
                ('depth_image', 'sensor_msgs/msg/Image[ignition.msgs.Image',
                 'depth/image_raw'),
                ('points', 'sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                 'points'),
                ('camera_info', 'sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                 'camera_info')):
            bridge_args.append(f'{gz_topic}/{suffix}@{msg}')
            bridge_remaps.append((f'{gz_topic}/{suffix}', f'{ros_topic}/{ros_suffix}'))

    if flags['camera'] == 'true':
        bridge_rgbd('front')
    if flags['side_cameras'] == 'true':
        for position in ('back', 'left', 'right'):
            bridge_rgbd(position)

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
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                spawner('antbot_swerve_controller'),
                spawner('lift_controller',
                        'left_arm_controller', 'right_arm_controller',
                        'left_gripper_controller', 'right_gripper_controller'),
            ])),
        gz_bridge,
    ]


def generate_launch_description():
    description_pkg = get_package_share_directory('antbot_description')
    openarm_pkg = get_package_share_directory('semi_humanoid_v2')

    # Meshes are package:// URIs into two packages, which an isolated colcon
    # install puts under different share/ dirs.
    resource_path = ':'.join(sorted({
        os.path.dirname(description_pkg), os.path.dirname(openarm_pkg)}))
    existing_resource = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    plugin_path = os.path.join('/opt', 'ros', os.environ.get('ROS_DISTRO', 'humble'), 'lib')
    existing_plugin = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='empty',
            description='World name (antbot_gazebo/config/worlds.yaml) or SDF path'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the Gazebo server without the GUI (ign gazebo -s)'),
        DeclareLaunchArgument(
            'lidar_3d', default_value='false',
            description='Simulated 3D LiDAR at the stock mount (-> /lidar_3d_points)'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Front S10 RGBD camera (-> /sensor/camera/stereo_front/*)'),
        DeclareLaunchArgument(
            'side_cameras', default_value='true',
            description='Back/left/right S10 RGBD cameras '
                        '(-> /sensor/camera/stereo_<position>/*)'),
        DeclareLaunchArgument(
            'calibration_yaml_path', default_value='',
            description='Optional sensor extrinsic yaml, same file the real robot '
                        'loads. Empty uses the nominal mounts.'),
        SetEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH',
            resource_path + (':' + existing_resource if existing_resource else '')),
        SetEnvironmentVariable(
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
            plugin_path + (':' + existing_plugin if existing_plugin else '')),
        OpaqueFunction(function=_resolve_world_path),
        OpaqueFunction(function=_robot_and_bridge),
    ])
