#!/usr/bin/env python3
"""
fleet_coordinator
=================

A minimal demonstration of INTER-ROBOT COMMUNICATION for the 3-robot warehouse
fleet, using a shared "fleet bus" built from global (un-namespaced) topics.

Why this pattern
----------------
Each robot lives in its own namespace (/robot1, /robot2, /robot3) so that Nav2,
TF and the controllers stay isolated. For the robots to *cooperate*, though,
they need a common channel that sits OUTSIDE those namespaces. The simplest,
most ROS-native way to do that is a set of shared topics under /fleet:

    /fleet/status     (this node -> everyone)   consolidated pose of every robot
    /fleet/<robot>/...                           per-robot coordination channels

This node:
  * subscribes to every robot's /robotN/odom,
  * republishes a single consolidated snapshot on /fleet/status (JSON), so any
    robot or dashboard can see the whole fleet from one topic, and
  * runs a tiny example of *coordination logic*: if two robots come within a
    safety distance it raises a conflict warning on /fleet/alerts. That is the
    hook where a real traffic-manager / task-allocator would live.

This is intentionally transport-only + a stub of logic: it shows HOW the robots
talk, and gives you one obvious place to add WHAT they decide.

Run:
    ros2 run warehouse_bot_package fleet_coordinator.py
"""

import json
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

ROBOTS = ['robot1', 'robot2', 'robot3']
SAFETY_DISTANCE = 0.6   # metres; closer than this -> conflict alert


class FleetCoordinator(Node):
    def __init__(self):
        super().__init__('fleet_coordinator')
        self._poses = {}    # robot -> (x, y, yaw)

        # Listen to every robot's odometry (each in its own namespace).
        for name in ROBOTS:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                self._make_odom_cb(name), 10)

        # Shared fleet channels (global, so every robot can read them).
        self._status_pub = self.create_publisher(String, '/fleet/status', 10)
        self._alert_pub = self.create_publisher(String, '/fleet/alerts', 10)

        self.create_timer(0.5, self._broadcast)   # 2 Hz
        self.get_logger().info(
            f'Fleet coordinator up. Tracking {ROBOTS} on the /fleet bus.')

    def _make_odom_cb(self, name):
        def cb(msg: Odometry):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._poses[name] = (p.x, p.y, yaw)
        return cb

    def _broadcast(self):
        if not self._poses:
            return

        # 1) Publish the consolidated fleet snapshot.
        snapshot = {n: {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(t, 3)}
                    for n, (x, y, t) in self._poses.items()}
        self._status_pub.publish(String(data=json.dumps(snapshot)))

        # 2) Coordination logic: flag pairs that are too close.
        names = list(self._poses)
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = self._poses[names[i]], self._poses[names[j]]
                d = math.hypot(a[0] - b[0], a[1] - b[1])
                if d < SAFETY_DISTANCE:
                    alert = {'type': 'proximity_conflict',
                             'robots': [names[i], names[j]],
                             'distance': round(d, 3)}
                    self._alert_pub.publish(String(data=json.dumps(alert)))
                    self.get_logger().warn(
                        f'CONFLICT: {names[i]} and {names[j]} are '
                        f'{d:.2f} m apart (< {SAFETY_DISTANCE} m). '
                        f'A traffic manager would slow/reroute one here.')


def main():
    rclpy.init()
    node = FleetCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
