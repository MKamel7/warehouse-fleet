#!/usr/bin/env python3
"""
TOP-LEVEL launch: full 3-robot autonomous warehouse simulation.

  1. Gazebo Classic + AWS small-warehouse world
  2. robot1 / robot2 / robot3 spawned, namespaced, white body + black wheels
  3. one Nav2 stack per robot (localized on the pre-built warehouse map)
  4. RViz pre-configured with all three robots

Usage:
    ros2 launch warehouse_bot_package bringup.launch.py
    ros2 launch warehouse_bot_package bringup.launch.py rviz:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_bot_package')
    launch_dir = os.path.join(pkg, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    rviz_config = os.path.join(pkg, 'rviz', 'multi_robot.rviz')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    # 1 + 2: world and robots
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'warehouse_multi_spawn.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 3: Nav2 stacks, started after the robots have spawned + TF is flowing.
    nav2 = TimerAction(period=12.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'nav2_multi.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])

    # 4: RViz. TF is per-namespace (/robot1/tf ...), so RViz is bound to one
    #    robot (robot1 by default) by remapping its tf onto that namespace.
    #    Gazebo is the "watch all three at once" view; for a per-robot RViz of
    #    robot2/robot3 use rviz.launch.py namespace:=robot2.
    rviz = TimerAction(period=14.0, actions=[
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', '/robot1/tf'),
                        ('/tf_static', '/robot1/tf_static')],
            output='screen',
        ),
    ])

    return LaunchDescription(declare + [sim, nav2, rviz])
