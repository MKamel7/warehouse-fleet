import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('warehouse_bot_package'),
        'urdf', 'warehouse_bot.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gazebo', '--verbose',
                 '/usr/share/gazebo-11/worlds/empty_sky.world',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_path,
                       '-entity', 'warehouse_bot',
                       '-x', '0', '-y', '0', '-z', '0.05'],
            output='screen'),
    ])
