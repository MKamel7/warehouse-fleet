#!/usr/bin/env python3
"""
LEADER-FOLLOWER CONVOY demo.

  * Gazebo + AWS warehouse + 3 robots (white body, black wheels)
  * robot1  = LEADER: full Nav2 stack (Regulated Pure Pursuit) -> drives to a goal
  * robot2/3 = FOLLOWERS: localization (AMCL) only; a convoy controller drives
               them along the leader's actual path at fixed spacing
  * RViz bound to the leader, showing the shared /convoy/leader_path

Usage:
    ros2 launch warehouse_bot_package convoy.launch.py
Then give the LEADER a goal (RViz "2D Goal Pose" -> /robot1/goal_pose, or
`ros2 run warehouse_bot_package fleet_send_goals.py` editing only robot1).
The other two will follow.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml

LEADER = 'robot1'
FOLLOWERS = ['robot2', 'robot3']


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_bot_package')
    nav2 = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(pkg, 'launch')

    map_yaml = os.path.join(pkg, 'maps', 'warehouse_map.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    rviz_config = os.path.join(pkg, 'rviz', 'multi_robot.rviz')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
    ]

    # 1) world + 3 robots
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'warehouse_multi_spawn.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'gui': LaunchConfiguration('gui')}.items())

    # 2) LEADER: real Nav2 PLANNER (global path on the warehouse map) + a smooth
    #    pure-pursuit controller (leader_controller.py). The stock RPP/rotation-
    #    shim controller oscillates at the 180-deg turn-around on this diff-drive
    #    robot, so we keep Nav2 planning but drive with pure pursuit. Localisation
    #    is ground-truth map->odom so the leader never gets lost.
    leader_params = os.path.join(pkg, 'params', 'nav2_leader.yaml')
    tf_remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    configured = RewrittenYaml(
        source_file=leader_params, root_key=LEADER,
        param_rewrites={'use_sim_time': 'true', 'yaml_filename': map_yaml},
        convert_types=True)
    leader_nav = TimerAction(period=12.0, actions=[
        GroupAction([
            PushRosNamespace(LEADER),
            Node(package='nav2_map_server', executable='map_server',
                 name='map_server', output='screen',
                 parameters=[configured], remappings=tf_remap),
            Node(package='nav2_planner', executable='planner_server',
                 name='planner_server', output='screen',
                 parameters=[configured], remappings=tf_remap),
            Node(package='nav2_lifecycle_manager',
                 executable='lifecycle_manager',
                 name='lifecycle_manager_leader', output='screen',
                 parameters=[{'use_sim_time': True, 'autostart': True,
                              'node_names': ['map_server', 'planner_server']}]),
        ]),
        # perfect map->odom from ground truth
        Node(package='warehouse_bot_package', executable='gt_localization.py',
             name='gt_localization_leader', output='screen',
             parameters=[{'robot': LEADER, 'use_sim_time': True}]),
        # pure-pursuit driver that follows the planner's path
        Node(package='warehouse_bot_package', executable='leader_controller.py',
             name='leader_controller', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])

    # 3) FOLLOWERS need no Nav2/AMCL: the convoy controller drives them directly
    #    from ground-truth poses. (Only the leader runs the full stack.)

    # 4) Convoy controller (drives the followers along the leader's path)
    convoy = TimerAction(period=18.0, actions=[
        Node(package='warehouse_bot_package',
             executable='convoy_controller.py',
             name='convoy_controller',
             output='screen',
             parameters=[{'use_sim_time': use_sim_time}])])

    # 5) RViz bound to the leader's namespaced tf. Started after the leader
    #    stack + ground-truth localisation are up, so the 'map' frame and the
    #    /robot1/map topic exist when RViz subscribes (esp. under GUI load).
    rviz = TimerAction(period=22.0, actions=[
        Node(condition=IfCondition(use_rviz),
             package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config],
             parameters=[{'use_sim_time': use_sim_time}],
             remappings=[('/tf', '/robot1/tf'), ('/tf_static', '/robot1/tf_static')],
             output='screen')])

    return LaunchDescription(declare + [sim, leader_nav, convoy, rviz])
