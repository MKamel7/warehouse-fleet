#!/usr/bin/env python3
"""
fleet_send_goals
================

Dispatch a navigation goal to every robot at once via each robot's namespaced
Nav2 action server (/robotN/navigate_to_pose). This is the "command" direction
of fleet communication (the coordinator / an operator telling robots where to
go), complementing fleet_coordinator.py which gathers fleet state.

Edit GOALS below (map-frame x, y, yaw) and run:
    ros2 run warehouse_bot_package fleet_send_goals.py
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# robot -> (x, y, yaw) goal in the map frame. Open spots in the warehouse aisle.
GOALS = {
    'robot1': (1.0, -1.0, 0.0),
    'robot2': (0.0, -2.5, 0.0),
    'robot3': (1.0, -4.0, 0.0),
}


def make_goal(x, y, yaw):
    goal = NavigateToPose.Goal()
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    goal.pose = p
    return goal


def main():
    rclpy.init()
    node = rclpy.create_node('fleet_send_goals')
    clients = {n: ActionClient(node, NavigateToPose, f'/{n}/navigate_to_pose')
               for n in GOALS}

    for name, (x, y, yaw) in GOALS.items():
        client = clients[name]
        node.get_logger().info(f'Waiting for {name} navigate_to_pose server...')
        if not client.wait_for_server(timeout_sec=10.0):
            node.get_logger().error(f'{name}: action server not available, skipping.')
            continue
        client.send_goal_async(make_goal(x, y, yaw))
        node.get_logger().info(f'{name}: goal sent -> ({x}, {y}, yaw={yaw}).')

    # Let the async goal requests flush, then exit (Nav2 keeps driving).
    rclpy.spin_once(node, timeout_sec=2.0)
    node.get_logger().info('All goals dispatched. Robots are navigating.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
