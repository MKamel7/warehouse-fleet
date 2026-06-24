#!/usr/bin/env python3
"""
Launch one full Nav2 stack per robot (robot1/2/3), each in its own namespace.

Each stack = map_server + amcl (localization) + planner + controller +
behaviors + bt_navigator + waypoint_follower + velocity_smoother, all reading
the per-robot params file (namespaced scan topic, base_link frames, and an
auto initial pose at the spawn location).

Assumes the simulation (warehouse_multi_spawn.launch.py) is already running,
or is started alongside this by bringup.launch.py.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROBOTS = ['robot1', 'robot2', 'robot3']


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_bot_package')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')

    map_yaml = os.path.join(pkg, 'maps', 'warehouse_map.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map', default_value=map_yaml),
    ]

    actions = list(declare)
    for i, ns in enumerate(ROBOTS):
        params_file = os.path.join(pkg, 'params', f'nav2_{ns}.yaml')
        stack = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                launch_arguments={
                    'namespace': ns,
                    'use_namespace': 'True',
                    'slam': 'False',
                    'map': LaunchConfiguration('map'),
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': autostart,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                }.items(),
            ),
        ])
        # Stagger nav2 stacks so 3 lifecycle managers don't fight for CPU at t=0.
        actions.append(TimerAction(period=float(2 * i), actions=[stack]))

    return LaunchDescription(actions)
