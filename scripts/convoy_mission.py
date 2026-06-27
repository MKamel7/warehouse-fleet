#!/usr/bin/env python3
"""Drive the leader through a list of waypoints (e.g. there and back).

The leader visits a sequence of stations in order and the followers come along;
optionally loops. Goals are published on /<leader>/goal_pose (same as RViz),
waiting for arrival at each waypoint before sending the next, so it sits on top
of leader_controller.py and its recovery. Waypoints are station names from
config/fleet.yaml (section 'mission') or given on the command line:

    ros2 run warehouse_bot_package convoy_mission.py                 # config
    ros2 run warehouse_bot_package convoy_mission.py pick_a home_1   # custom
    ros2 run warehouse_bot_package convoy_mission.py pick_a drop_x --loop
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import wb_common as wb


class ConvoyMission(Node):
    def __init__(self, waypoints, loop):
        super().__init__('convoy_mission')

        cfg = wb.load_fleet_config()
        self.leader = cfg['convoy']['leader']
        stations = cfg['stations']
        mission = cfg.get('mission', {})

        names = waypoints or mission.get('waypoints', [])
        bad = [n for n in names if n not in stations]
        if bad:
            raise SystemExit(
                f'unknown station(s) {bad}; known: {list(stations)}')
        if len(names) < 1:
            raise SystemExit('no waypoints given (and none in config mission).')

        self.waypoints = [(n, stations[n]) for n in names]
        self.loop = loop if loop is not None else mission.get('loop', False)
        self.tol = mission.get('arrive_tol', 0.35)
        self.pause = mission.get('pause', 1.0)

        self.pose = None
        self.idx = 0
        self.sent = False
        self.dwell = 0.0
        self.done = False

        self.create_subscription(Odometry, f'/{self.leader}/ground_truth',
                                 self._pose_cb, 20)
        self.goal_pub = self.create_publisher(
            PoseStamped, f'/{self.leader}/goal_pose', 10)
        self.dt = 0.2
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'mission: {[n for n, _ in self.waypoints]} '
            f'(loop={self.loop}) on leader {self.leader}.')

    def _pose_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _send(self):
        name, st = self.waypoints[self.idx]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(st['x'])
        msg.pose.position.y = float(st['y'])
        z, w = wb.quat_from_yaw(float(st.get('yaw', 0.0)))
        msg.pose.orientation.z = z
        msg.pose.orientation.w = w
        self.goal_pub.publish(msg)
        self.sent = True
        self.get_logger().info(
            f'-> waypoint {self.idx + 1}/{len(self.waypoints)}: {name} '
            f'({st["x"]}, {st["y"]})')

    def _tick(self):
        if self.done or self.pose is None:
            return
        if not self.sent:
            self._send()                       # (re)send current waypoint
            return
        name, st = self.waypoints[self.idx]
        if not wb.reached(self.pose, (st['x'], st['y']), self.tol):
            return
        # arrived: dwell, then advance
        self.dwell += self.dt
        if self.dwell < self.pause:
            return
        self.dwell = 0.0
        self.get_logger().info(f'reached {name}.')
        nxt = wb.mission_advance(self.idx, len(self.waypoints), self.loop)
        if nxt is None:
            self.get_logger().info('mission complete.')
            self.done = True
            return
        self.idx = nxt
        self.sent = False


def main():
    parser = argparse.ArgumentParser(description='Run a convoy waypoint tour.')
    parser.add_argument('waypoints', nargs='*',
                        help='station names in order (default: config mission)')
    parser.add_argument('--loop', action='store_true', help='repeat forever')
    # tolerate ROS args appended by ros2 launch/run
    args, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init()
    node = ConvoyMission(args.waypoints, args.loop or None)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
