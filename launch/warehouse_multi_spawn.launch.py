#!/usr/bin/env python3
"""
Bring up the AWS small-warehouse world in Gazebo Classic and spawn 3
namespaced warehouse_bot robots (robot1, robot2, robot3).

Each robot gets:
  * its own robot_state_publisher (namespaced  ->  /robotN/tf, /robotN/tf_static)
  * a spawn into Gazebo at a distinct pose in the open aisle (x~2, y -1..-4)
  * namespaced topics: /robotN/cmd_vel, /robotN/odom, /robotN/scan, ...

The namespace is baked into each robot's URDF (xacro `namespace` arg) so the
Gazebo plugins publish under /robotN/...  We expand the xacro here in Python
and spawn with `-file` (spawn_entity.py has no -string, and PushRosNamespace
does not reach into a nested TimerAction, so -topic is unreliable here).

This launch is the SIMULATION layer only (world + robots). Navigation is added
on top by nav2_multi.launch.py / bringup.launch.py.
"""

import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def load_robots(pkg):
    """Fleet roster (name + spawn pose) from config/fleet.yaml -> scales the
    whole launch with the config."""
    with open(os.path.join(pkg, 'config', 'fleet.yaml')) as f:
        return yaml.safe_load(f)['fleet']['robots']


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_bot_package')
    aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    robots = load_robots(pkg)

    xacro_file = os.path.join(pkg, 'urdf', 'warehouse_bot.urdf.xacro')
    world = os.path.join(aws, 'worlds', 'no_roof_small_warehouse',
                        'no_roof_small_warehouse.world')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Make the AWS warehouse discoverable by Gazebo.
    #  * GAZEBO_MODEL_PATH   -> resolves the world's  model://...  includes
    #  * GAZEBO_RESOURCE_PATH-> resolves each model's file://models/... meshes
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(aws, 'models') + os.pathsep
        + os.environ.get('GAZEBO_MODEL_PATH', ''))
    set_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=aws + os.pathsep + os.environ.get('GAZEBO_RESOURCE_PATH', ''))

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('gui', default_value='true'),
    ]

    # --- Gazebo server + client (with the ROS factory plugin) -----------
    gzserver = ExecuteProcess(
        cmd=['gzserver', LaunchConfiguration('world'),
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '--verbose'],
        output='screen')
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen')

    actions = list(declare_args) + [set_model_path, set_resource_path,
                                    gzserver, gzclient]

    # Expand the xacro once per robot and cache the URDF on disk for spawning.
    urdf_dir = os.path.join(tempfile.gettempdir(), 'warehouse_bot_urdf')
    os.makedirs(urdf_dir, exist_ok=True)

    for i, r in enumerate(robots):
        ns = r['name']
        # ROS namespace must NOT have a trailing slash (rclcpp rejects it).
        doc = xacro.process_file(xacro_file, mappings={'namespace': ns})
        urdf_xml = doc.toxml()
        urdf_path = os.path.join(urdf_dir, ns + '.urdf')
        with open(urdf_path, 'w') as f:
            f.write(urdf_xml)

        group = GroupAction([
            PushRosNamespace(ns),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': urdf_xml,
                             'use_sim_time': use_sim_time,
                             'frame_prefix': ''}],
                # Publish TF into this robot's namespace (/robotN/tf) instead of
                # the global /tf, so the three robots don't collide on shared
                # odom/base_link frames. Matches Nav2 namespaced bringup.
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            ),
            # Stagger the spawns so Gazebo's factory handles them cleanly.
            TimerAction(period=float(3 + 2 * i), actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_' + ns,
                    output='screen',
                    arguments=[
                        '-entity', ns,
                        '-file', urdf_path,
                        '-x', str(r['x']),
                        '-y', str(r['y']),
                        '-z', '0.05',
                        '-Y', str(r['yaw']),
                    ],
                ),
            ]),
        ])
        actions.append(group)

    return LaunchDescription(actions)
