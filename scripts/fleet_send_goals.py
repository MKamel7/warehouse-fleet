#!/usr/bin/env python3
"""Send a navigation goal to every robot at once.

Each robot is sent 3 m down the aisle (-x from its spawn), derived from
config/fleet.yaml so it scales with the roster. For pick->drop dispatch use
task_allocator.py + submit_task.py instead. Needs the full Nav2 stack per robot
(bringup.launch.py); the convoy demo runs only the planner, so use the
/robot1/goal_pose topic there.
"""

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

import wb_common as wb


def make_goal(x, y, yaw):
    goal = NavigateToPose.Goal()
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    z, w = wb.quat_from_yaw(yaw)
    p.pose.orientation.z = z
    p.pose.orientation.w = w
    goal.pose = p
    return goal


def main():
    rclpy.init()
    node = rclpy.create_node('fleet_send_goals')

    cfg = wb.load_fleet_config()
    spawns = wb.robot_spawns(cfg)
    # default demo goal: 3 m down the aisle (-x) from each robot's spawn.
    goals = {n: (x - 3.0, y, yaw) for n, (x, y, yaw) in spawns.items()}

    clients = {n: ActionClient(node, NavigateToPose, f'/{n}/navigate_to_pose')
               for n in goals}

    for name, (x, y, yaw) in goals.items():
        client = clients[name]
        node.get_logger().info(f'Waiting for {name} navigate_to_pose server...')
        if not client.wait_for_server(timeout_sec=10.0):
            node.get_logger().error(f'{name}: action server not available, skipping.')
            continue
        client.send_goal_async(make_goal(x, y, yaw))
        node.get_logger().info(f'{name}: goal sent -> ({x:.2f}, {y:.2f}, yaw={yaw}).')

    rclpy.spin_once(node, timeout_sec=2.0)
    node.get_logger().info('All goals dispatched. Robots are navigating.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
