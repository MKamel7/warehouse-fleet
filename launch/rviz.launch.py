#!/usr/bin/env python3
"""
Open an RViz bound to a single robot's namespaced TF tree.

    ros2 launch warehouse_bot_package rviz.launch.py namespace:=robot2

Run one per robot you want to watch in RViz. (Gazebo already shows all three.)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_bot_package')
    rviz_config = os.path.join(pkg, 'rviz', 'multi_robot.rviz')
    ns = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/tf', PythonExpression(["'/' + '", ns, "' + '/tf'"])),
                ('/tf_static', PythonExpression(["'/' + '", ns, "' + '/tf_static'"])),
            ],
            output='screen',
        ),
    ])
