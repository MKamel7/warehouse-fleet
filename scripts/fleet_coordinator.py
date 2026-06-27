#!/usr/bin/env python3
"""Fleet state plus traffic management over a shared /fleet topic bus.

Each robot is namespaced, so cooperation uses global /fleet topics:
    /fleet/status   consolidated pose of every robot (JSON)
    /fleet/alerts   proximity-conflict events (JSON)
    /fleet/hold     list of robots told to pause (JSON)

Poses come from each robot's map-frame /robotN/ground_truth (not /odom, whose
origin is each robot's own spawn point, so distances there aren't comparable).
When two robots get too close the lower-priority one (later in the roster) is
held until they separate; separate warn/clear distances give hysteresis so the
hold doesn't flap. convoy_controller and task_allocator both obey /fleet/hold.
"""

import json

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import wb_common as wb


class FleetCoordinator(Node):
    def __init__(self):
        super().__init__('fleet_coordinator')

        cfg = wb.load_fleet_config()
        self.robots = wb.robot_names(cfg)
        self.priority = {n: i for i, n in enumerate(self.robots)}  # lower = wins
        self.WARN = cfg['safety']['proximity_warn']
        self.CLEAR = cfg['safety']['proximity_clear']

        self._poses = {}    # robot -> (x, y, yaw)  in the map frame
        self._held = set()  # robots currently told to hold (hysteresis state)

        for name in self.robots:
            self.create_subscription(
                Odometry, f'/{name}/ground_truth',
                self._make_pose_cb(name), 10)

        self._status_pub = self.create_publisher(String, '/fleet/status', 10)
        self._alert_pub = self.create_publisher(String, '/fleet/alerts', 10)
        self._hold_pub = self.create_publisher(String, '/fleet/hold', 10)

        self.create_timer(0.5, self._tick)   # 2 Hz
        self.get_logger().info(
            f'Fleet coordinator + traffic manager up. Tracking {self.robots} '
            f'(warn<{self.WARN} m, clear>{self.CLEAR} m).')

    def _make_pose_cb(self, name):
        def cb(msg: Odometry):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self._poses[name] = (p.x, p.y, wb.yaw_from_quat(q.x, q.y, q.z, q.w))
        return cb

    def _tick(self):
        if not self._poses:
            return

        # 1) consolidated fleet snapshot
        snapshot = {n: {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(t, 3)}
                    for n, (x, y, t) in self._poses.items()}
        self._status_pub.publish(String(data=json.dumps(snapshot)))

        # 2) traffic management with warn/clear hysteresis
        still_held = wb.conflict_holds(self._poses, self.priority,
                                       self.WARN, self.CLEAR, self._held)
        for r in still_held:
            self._alert_pub.publish(String(data=json.dumps({
                'type': 'proximity_conflict', 'hold': r})))

        if still_held != self._held:
            for r in still_held - self._held:
                self.get_logger().warn(
                    f'CONFLICT: holding {r} (lower priority) to resolve traffic.')
            for r in self._held - still_held:
                self.get_logger().info(f'CLEARED: releasing {r}.')
            self._held = still_held
        # publish the current hold set every tick so late subscribers stay in sync
        self._hold_pub.publish(String(data=json.dumps(sorted(self._held))))


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
