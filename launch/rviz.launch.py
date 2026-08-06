#!/usr/bin/env python3
"""
Open an RViz bound to a single robot, in its own window.

    ros2 launch warehouse_bot_package rviz.launch.py namespace:=robot2

The shipped config (rviz/multi_robot.rviz) is written against robot1's topics
and TF. This launch remaps robot1's TF *and* every /robot1/<topic> the config
uses onto the requested namespace, so the window shows that robot's map, model,
scan, costmap and plan, and its "2D Goal Pose" tool commands that robot.
Run one per robot you want to watch/command. (Gazebo already shows all three.)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The /robot1/<topic> references baked into rviz/multi_robot.rviz. Keep in sync
# if you add displays/tools that subscribe or publish on a robot1 topic.
CONFIG_TOPICS = [
    'map',
    'map_updates',                      # companion topic the Map display subscribes to
    'robot_description',
    'scan',
    'global_costmap/costmap',
    'global_costmap/costmap_updates',   # companion topic for the costmap display
    'plan',
    'initialpose',
    'goal_pose',
]


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('warehouse_bot_package')
    rviz_config = os.path.join(pkg, 'rviz', 'multi_robot.rviz')
    ns = LaunchConfiguration('namespace').perform(context)

    # TF first, then each topic from the config. For namespace:=robot1 these are
    # no-op self-remaps, so the default still works unchanged.
    remaps = [('/tf', f'/{ns}/tf'), ('/tf_static', f'/{ns}/tf_static')]
    remaps += [(f'/robot1/{t}', f'/{ns}/{t}') for t in CONFIG_TOPICS]

    return [Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=remaps,
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
