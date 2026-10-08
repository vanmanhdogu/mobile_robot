"""robot_sensor_gazebo.launch.py with the two RoboSense Airy 3D LiDARs mounted.

Spawns semi_humanoid_v2_airy.urdf.xacro instead of semi_humanoid_v2_real.urdf.xacro;
everything else (world, controllers, sensor flags, bridge table) is taken from
robot_sensor_gazebo.launch.py itself, so the two stay in step. Adds:

  sensor                 arg (default)                ROS topic
  Airy 3D LiDAR, front   airy (true)                  /airy_points        (PointCloud2, 10 Hz)
  Airy 3D LiDAR, rear    airy_back (true)             /airy_back_points   (PointCloud2, 10 Hz)
                         airy_vertical_samples (96)   192 / 96 / 48 lines, both units

The two hemispheres face opposite ways (front dome +x, rear dome -x), so
together they see the whole sphere around the robot. Each unit costs a full
gpu_lidar; on a Jetson drop airy_vertical_samples to 48 if the sim falls behind.

All arguments of robot_sensor_gazebo.launch.py (world, headless,
camera, side_cameras, calibration_yaml_path) work unchanged.
"""

import importlib.util
import os
import types

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_base_launch():
    """Import robot_sensor_gazebo.launch.py as a module."""
    path = os.path.join(get_package_share_directory('semi_humanoid_v2'),
                        'launch', 'robot_sensor_gazebo.launch.py')
    spec = importlib.util.spec_from_file_location('robot_sensor_gazebo_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


base = _load_base_launch()
_base_robot_and_bridge = base._robot_and_bridge


def _robot_and_bridge_airy(context, *args, **kwargs):
    """Run the base spawn/bridge step on the Airy xacro, then bridge the Airy."""
    airy = LaunchConfiguration('airy').perform(context).lower()
    airy_back = LaunchConfiguration('airy_back').perform(context).lower()
    airy_mappings = {
        'airy': airy,
        'airy_back': airy_back,
        'airy_vertical_samples':
            LaunchConfiguration('airy_vertical_samples').perform(context),
    }
    airy_xacro = os.path.join(get_package_share_directory('semi_humanoid_v2'),
                              'urdf', 'semi_humanoid_v2_airy.urdf.xacro')

    # The base step builds the URDF with xacro.process_file(<real xacro>, ...);
    # swap in the Airy xacro (which includes the real one) for that one call.
    real_xacro = base.xacro
    base.xacro = types.SimpleNamespace(
        process_file=lambda _path, mappings: real_xacro.process_file(
            airy_xacro, mappings={**mappings, **airy_mappings}))
    try:
        actions = _base_robot_and_bridge(context, *args, **kwargs)
    finally:
        base.xacro = real_xacro

    # Only the point clouds: one ring of a 3D scan is not useful as a LaserScan.
    for name, enabled in (('airy', airy), ('airy_back', airy_back)):
        if enabled != 'true':
            continue
        actions.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{name}_bridge',
            arguments=[f'/{name}/points@sensor_msgs/msg/PointCloud2'
                       f'[ignition.msgs.PointCloudPacked'],
            remappings=[(f'/{name}/points', f'/{name}_points')],
            parameters=[{'use_sim_time': True}],
            output='screen'))
    return actions


def generate_launch_description():
    # generate_launch_description() looks _robot_and_bridge up at call time.
    base._robot_and_bridge = _robot_and_bridge_airy
    # The Airy arguments go first: the base description ends with the
    # OpaqueFunction that reads them.
    return LaunchDescription([
        DeclareLaunchArgument(
            'airy', default_value='true',
            description='RoboSense Airy 3D LiDAR on the column front (-> /airy_points)'),
        DeclareLaunchArgument(
            'airy_back', default_value='true',
            description='Second Airy on the column rear, facing the opposite way '
                        '(-> /airy_back_points)'),
        DeclareLaunchArgument(
            'airy_vertical_samples', default_value='96',
            description='Airy lines per unit: 192, 96 or 48. 192 may not hold '
                        'real-time on a Jetson, more so with both units on.'),
        *base.generate_launch_description().entities,
    ])
