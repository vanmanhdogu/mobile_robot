"""One-command bringup: ANTBot + OpenArm robot, Nav2 (or SLAM), and RViz.

  mode:=sim   antbot_openarm_description/gazebo.launch.py in Gazebo
  mode:=real  antbot_openarm_description/real_robot.launch.py plus the 2D LiDAR
              driver (antbot_bringup/lidar_2d.launch.py), which Nav2 needs

  slam:=false navigation.launch.py: map_server + AMCL + Nav2 on a saved map
              (world:= looks the map up in maps/worlds.yaml; map:= overrides)
  slam:=true  slam.launch.py: slam_toolbox builds a map; drive with teleop or
              /cmd_vel and save it with nav2_map_server map_saver_cli

Navigation starts nav_delay seconds after the robot, so the controllers are up
and odom -> base_link exists before AMCL or slam_toolbox look for it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, launch_file, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments=launch_arguments.items())


def _bringup(context, *args, **kwargs):
    arg = {name: LaunchConfiguration(name).perform(context) for name in (
        'mode', 'slam', 'world', 'map', 'rviz', 'headless',
        'use_mock_hardware', 'nav_delay')}
    sim = arg['mode'] == 'sim'
    slam = arg['slam'].lower() == 'true'

    actions = []

    # ── robot ──
    if sim:
        actions.append(_include('antbot_openarm_description', 'gazebo.launch.py', {
            'world': arg['world'] or 'empty',
            'headless': arg['headless'],
        }))
    else:
        actions.append(_include('antbot_openarm_description', 'real_robot.launch.py', {
            'use_mock_hardware': arg['use_mock_hardware'],
        }))
        # Nav2 and SLAM need /scan_0; real_robot.launch.py starts no sensors.
        actions.append(_include('antbot_bringup', 'lidar_2d.launch.py', {}))

    # ── navigation or SLAM ──
    if slam:
        nav = _include('semi_humanoid_navigation', 'slam.launch.py',
                       {'mode': arg['mode']})
    else:
        nav_args = {'mode': arg['mode']}
        if arg['map']:
            nav_args['map'] = arg['map']
        else:
            nav_args['world'] = arg['world']
        nav = _include('semi_humanoid_navigation', 'navigation.launch.py', nav_args)
    actions.append(TimerAction(period=float(arg['nav_delay']), actions=[nav]))

    # ── RViz ──
    if arg['rviz'].lower() == 'true':
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('semi_humanoid_navigation'),
                'rviz', 'navigation.rviz')],
            parameters=[{'use_sim_time': sim}]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sim', choices=['sim', 'real'],
            description='sim: Gazebo, real: physical robot'),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='true: build a map with slam_toolbox instead of navigating'),
        DeclareLaunchArgument(
            'world', default_value='depot',
            description='Gazebo world (sim), and the map to navigate on via '
                        'maps/worlds.yaml'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Map YAML to navigate on; overrides the world lookup'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz with rviz/navigation.rviz'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='sim only: run Gazebo without its window'),
        DeclareLaunchArgument(
            'use_mock_hardware', default_value='false',
            description='real only: mock the base (no ANT-RCU board)'),
        DeclareLaunchArgument(
            'nav_delay', default_value='15.0',
            description='Seconds to wait for the robot before starting '
                        'navigation or SLAM'),
        OpaqueFunction(function=_bringup),
    ])
