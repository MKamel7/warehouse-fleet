#!/usr/bin/env python3
"""
convoy_controller  -  leader/follower formation for the warehouse fleet
=======================================================================

ONE leader (robot1) drives autonomously with Nav2. The other robots
(robot2, robot3) are NOT given their own Nav2 goals; instead they trace the
exact path the leader actually travelled, each holding a fixed gap behind the
one in front. Because they replay the leader's collision-free, Nav2-planned
route, the whole convoy inherits the "best path" the leader found and stays
clear of obstacles, while a pure-pursuit law gives smooth execution.

How it works
------------
* The leader is localised by its own Nav2/AMCL; followers run AMCL too, so all
  poses are available in the shared `map` frame on /<robot>/amcl_pose.
* As the leader moves we append its pose to a breadcrumb TRAIL with cumulative
  arc-length s.
* Follower i has a target slot at arc-length  s_target = s_leader - gap_i.
  - Longitudinal: v = v_leader_feedforward + Kp * (s_target - s_follower)
    so it matches the leader's speed and closes any spacing error.
  - Lateral/heading: pure pursuit toward a look-ahead point further along the
    trail, so the follower stays ON the path instead of cutting corners.
* When the leader stops (goal reached) the followers close up to their slots
  and stop -> the convoy parks in formation.

Publishes the leader trail on /convoy/leader_path (nav_msgs/Path) for RViz.

Run:
    ros2 run warehouse_bot_package convoy_controller.py
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path

LEADER = 'robot1'
# follower name -> gap (metres) to keep behind the leader along the trail
FOLLOWERS = {'robot2': 0.9, 'robot3': 1.8}
# the robot immediately in front of each follower (for collision/spacing safety)
AHEAD = {'robot2': 'robot1', 'robot3': 'robot2'}
MIN_GAP = 0.6          # trail spacing: keep this far behind the robot ahead (m)
COLLISION_STOP = 0.35  # hard anti-collision stop vs ANY other robot (m)
YIELD_DIST = 1.5       # start yielding when the leader is this close and incoming
YIELD_STEP = 0.85      # how far to pull aside out of the leader's way (m)

# control gains / limits
KP_LONG = 0.9          # spacing error -> linear vel
KP_YAW = 1.8           # heading error -> angular vel
V_MAX = 0.26
W_MAX = 1.2
LOOKAHEAD = 0.45       # pure-pursuit look-ahead along the trail (m)
TRAIL_MIN_STEP = 0.05  # append a breadcrumb every 5 cm of leader travel
GOAL_SETTLE = 0.10     # follower considered "in slot" within 10 cm
LEADER_STOP_SPEED = 0.03


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Convoy(Node):
    def __init__(self):
        super().__init__('convoy_controller')

        self.pose = {}                 # robot -> (x, y, yaw)
        self.trail = deque(maxlen=4000)  # list of (x, y, s)
        self.fidx = {f: 0 for f in FOLLOWERS}   # follower index on the trail
        self.leader_speed = 0.0
        self._last_leader_s = 0.0

        # Continuous ground-truth pose (map frame) from each robot's p3d plugin.
        for name in [LEADER] + list(FOLLOWERS):
            self.create_subscription(
                Odometry, f'/{name}/ground_truth',
                self._make_pose_cb(name), 20)

        self.cmd_pub = {f: self.create_publisher(Twist, f'/{f}/cmd_vel', 10)
                        for f in FOLLOWERS}
        self.path_pub = self.create_publisher(Path, '/convoy/leader_path', 1)

        self.dt = 0.05
        self.create_timer(self.dt, self._control)
        self.get_logger().info(
            f'Convoy up. Leader={LEADER}, followers={FOLLOWERS} '
            f'(gaps in m). Drive the leader with a Nav2 goal.')

    def _make_pose_cb(self, name):
        def cb(msg: Odometry):
            p = msg.pose.pose
            self.pose[name] = (p.position.x, p.position.y, yaw_of(p.orientation))
            if name == LEADER:
                self._extend_trail(p.position.x, p.position.y)
        return cb

    def _extend_trail(self, x, y):
        if not self.trail:
            self.trail.append((x, y, 0.0))
            return
        lx, ly, ls = self.trail[-1]
        d = math.hypot(x - lx, y - ly)
        if d >= TRAIL_MIN_STEP:
            self.trail.append((x, y, ls + d))

    def _leader_s(self):
        return self.trail[-1][2] if self.trail else 0.0

    def _advance_index(self, name):
        """Snap the follower's trail index to the nearest trail point (allowing
        a little backward slack, then the whole forward trail)."""
        if len(self.trail) < 2 or name not in self.pose:
            return
        fx, fy, _ = self.pose[name]
        start = max(0, self.fidx[name] - 20)
        best, bestd = start, float('inf')
        for j in range(start, len(self.trail)):
            d = (fx - self.trail[j][0]) ** 2 + (fy - self.trail[j][1]) ** 2
            if d < bestd:
                bestd, best = d, j
        self.fidx[name] = best

    def _point_at_s(self, target_s):
        """Linear-interpolate a trail point at arc-length target_s."""
        if not self.trail:
            return None
        if target_s <= 0:
            return self.trail[0][0], self.trail[0][1]
        for k in range(1, len(self.trail)):
            if self.trail[k][2] >= target_s:
                x0, y0, s0 = self.trail[k - 1]
                x1, y1, s1 = self.trail[k]
                r = (target_s - s0) / max(s1 - s0, 1e-6)
                return x0 + r * (x1 - x0), y0 + r * (y1 - y0)
        return self.trail[-1][0], self.trail[-1][1]

    def _yield_if_leader_incoming(self, name, fx, fy, fyaw):
        """If the leader is heading toward this follower, pull perpendicular out
        of its path so it can pass, then return True (skip normal following)."""
        if LEADER not in self.pose:
            return False
        lx, ly, lyaw = self.pose[LEADER]
        d_lead = math.hypot(fx - lx, fy - ly)
        if d_lead > YIELD_DIST:
            return False
        # is this follower in front of the leader (leader driving at it)?
        rel = norm(math.atan2(fy - ly, fx - lx) - lyaw)
        if abs(rel) > 1.0:
            return False
        # step aside perpendicular to the leader's heading, to the side we're on
        side = 1.0 if rel >= 0 else -1.0
        ah = lyaw + side * (math.pi / 2.0)
        tx, ty = fx + YIELD_STEP * math.cos(ah), fy + YIELD_STEP * math.sin(ah)
        herr = norm(math.atan2(ty - fy, tx - fx) - fyaw)
        cmd = Twist()
        cmd.angular.z = max(-W_MAX, min(W_MAX, KP_YAW * herr))
        cmd.linear.x = 0.18 if abs(herr) < 1.0 else 0.0
        self.cmd_pub[name].publish(cmd)
        return True

    def _control(self):
        # estimate leader speed (for feed-forward)
        s = self._leader_s()
        self.leader_speed = 0.8 * self.leader_speed \
            + 0.2 * abs(s - self._last_leader_s) / self.dt
        self._last_leader_s = s

        self._publish_path()

        if len(self.trail) < 2:
            return

        for name, gap in FOLLOWERS.items():
            if name not in self.pose:
                continue
            fx, fy, fyaw = self.pose[name]

            # YIELD: if the leader is driving toward this follower (e.g. heading
            # back through the convoy), step aside so it can pass, then re-form.
            if self._yield_if_leader_incoming(name, fx, fy, fyaw):
                continue

            # Hold position until the leader is at least one gap ahead, so the
            # followers never crowd the leader (which would block its sensors).
            if s < gap:
                self.cmd_pub[name].publish(Twist())
                continue
            self._advance_index(name)
            s_follower = self.trail[self.fidx[name]][2]
            s_target = max(0.0, s - gap)

            # look-ahead point on the trail, clamped so we never aim past leader
            look_s = min(s_follower + LOOKAHEAD, s)
            tgt = self._point_at_s(look_s)
            if tgt is None:
                continue
            tx, ty = tgt

            # heading control (pure pursuit)
            heading_err = norm(math.atan2(ty - fy, tx - fx) - fyaw)
            w = max(-W_MAX, min(W_MAX, KP_YAW * heading_err))

            # longitudinal control: feed-forward leader speed + spacing error
            spacing_err = s_target - s_follower
            v = self.leader_speed + KP_LONG * spacing_err
            if abs(heading_err) > 0.8:
                v = min(v, 0.05)
            v = max(0.0, min(V_MAX, v))

            # SAFETY: never crowd the robot directly ahead. Scale speed down as
            # we approach MIN_GAP and stop (or creep) at it. This also stops a
            # follower from blocking the leader as it settles on its goal.
            ahead = AHEAD.get(name)
            if ahead in self.pose:
                ax, ay, _ = self.pose[ahead]
                d_ahead = math.hypot(ax - fx, ay - fy)
                if d_ahead < MIN_GAP:
                    v = 0.0
                elif d_ahead < MIN_GAP + 0.4:
                    v = min(v, V_MAX * (d_ahead - MIN_GAP) / 0.4)

            # SAFETY 2: hard anti-collision against EVERY other robot. Only
            # triggers at close range (COLLISION_STOP) so normal ~0.9 m convoy
            # spacing and re-forming are NOT frozen -> prevents crashes when
            # paths cross without deadlocking the formation.
            for other, (ox, oy, _) in self.pose.items():
                if other == name:
                    continue
                d = math.hypot(ox - fx, oy - fy)
                if d < COLLISION_STOP and abs(norm(math.atan2(oy - fy, ox - fx) - fyaw)) < math.pi / 2:
                    v = 0.0

            # park in formation when the leader has stopped and we're in slot
            if self.leader_speed < LEADER_STOP_SPEED and spacing_err < GOAL_SETTLE:
                v, w = 0.0, 0.0

            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_pub[name].publish(cmd)

    def _publish_path(self):
        if not self.trail:
            return
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, _ in self.trail:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)


def main():
    rclpy.init()
    node = Convoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # stop the followers on exit
        for pub in node.cmd_pub.values():
            pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
