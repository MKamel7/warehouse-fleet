#!/usr/bin/env python3
"""
Full 3-robot autonomous warehouse simulation.

  1. Gazebo Classic + AWS small-warehouse world
  2. robotN spawned (roster from config/fleet.yaml), namespaced, white + black
  3. one Nav2 stack per robot (localized on the pre-built warehouse map)
  4. RViz pre-configured (bound to the first robot)
  5. fleet_coordinator (fleet bus + traffic management) and, by default,
     task_allocator (warehouse pick->drop dispatch)

Usage:
    ros2 launch warehouse_bot_package bringup.launch.py
    ros2 launch warehouse_bot_package bringup.launch.py rviz:=false
    ros2 launch warehouse_bot_package bringup.launch.py tasks:=false
Then, e.g.:
    ros2 run warehouse_bot_package submit_task.py pick_a drop_x
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
        DeclareLaunchArgument('tasks', default_value='true',
                              description='run the task allocator'),
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

    # 5: fleet bus + traffic management, and (optionally) task allocation.
    #    Started after Nav2 so the navigate_to_pose servers exist for dispatch.
    fleet = TimerAction(period=16.0, actions=[
        Node(package='warehouse_bot_package', executable='fleet_coordinator.py',
             name='fleet_coordinator', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(condition=IfCondition(LaunchConfiguration('tasks')),
             package='warehouse_bot_package', executable='task_allocator.py',
             name='task_allocator', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
    ])

    return LaunchDescription(declare + [sim, nav2, rviz, fleet])
