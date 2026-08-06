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
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def load_robot_names(pkg):
    with open(os.path.join(pkg, 'config', 'fleet.yaml')) as f:
        return [r['name'] for r in yaml.safe_load(f)['fleet']['robots']]


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_bot_package')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
    robots = load_robot_names(pkg)

    map_yaml = os.path.join(pkg, 'maps', 'warehouse_map.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map', default_value=map_yaml),
    ]
    # Seconds between starting each robot's Nav2 stack. Spacing them out lets
    # each stack lifecycle-activate before the next starts, avoiding the
    # "Failed to change state ... get_state service ... async_send_request
    # failed" races when all stacks configure at once under load. Override with
    # WAREHOUSE_NAV2_STAGGER if your machine needs more/less. 15 s reliably
    # brings up all three stacks here; lower it on a faster machine to start
    # quicker, raise it if a robot's stack still stalls during bringup.
    stagger = float(os.environ.get('WAREHOUSE_NAV2_STAGGER', '15.0'))

    actions = list(declare)
    for i, ns in enumerate(robots):
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
        # Stagger nav2 stacks so the lifecycle managers don't all configure +
        # activate at the same time (which times out get_state services).
        actions.append(TimerAction(period=stagger * i, actions=[stack]))

    return LaunchDescription(actions)
