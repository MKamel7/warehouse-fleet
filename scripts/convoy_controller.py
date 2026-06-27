#!/usr/bin/env python3
"""Leader/follower convoy controller.

The leader drives with Nav2; the followers trace the leader's travelled path,
each holding a fixed gap behind the one in front. We record the leader's pose
as a breadcrumb trail with cumulative arc-length s; follower i targets
s_target = s_leader - gap_i, using feed-forward speed plus a spacing term for
distance and pure pursuit for heading. When the leader stops they close up and
park.

On top of that: keep clear of the robot ahead (min_gap), hard stop near any
robot (collision_stop), slow/stop for obstacles seen on the follower's own
/scan and side-step around persistent ones, and obey fleet holds (/fleet/hold).

Config: config/fleet.yaml. Publishes the trail on /convoy/leader_path.
"""

import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import wb_common as wb

YIELD_DIST = 1.5       # start yielding when the leader is this close and incoming
YIELD_STEP = 0.85      # how far to pull aside out of the leader's way (m)
GOAL_SETTLE = 0.10     # follower considered "in slot" within 10 cm
LEADER_STOP_SPEED = 0.03
OBSTACLE_CONE = math.pi / 6.0   # +-30 deg forward sector checked on /scan


class Convoy(Node):
    def __init__(self):
        super().__init__('convoy_controller')

        cfg = wb.load_fleet_config()
        self.LEADER = cfg['convoy']['leader']
        self.FOLLOWERS = wb.convoy_followers(cfg)   # name -> gap
        self.AHEAD = wb.convoy_ahead(cfg)           # name -> robot in front
        ctl, saf = cfg['control'], cfg['safety']
        self.KP_LONG = ctl['kp_long']
        self.KP_YAW = ctl['kp_yaw']
        self.V_MAX = ctl['v_max']
        self.W_MAX = ctl['w_max']
        self.LOOKAHEAD = ctl['lookahead']
        self.TRAIL_MIN_STEP = ctl['trail_min_step']
        self.MIN_GAP = saf['min_gap']
        self.COLLISION_STOP = saf['collision_stop']
        self.OBSTACLE_STOP = saf['obstacle_stop']
        self.OBSTACLE_SLOW = saf['obstacle_slow']
        rc = cfg['recovery']
        self.BLOCKED_TIME = rc['blocked_time']
        self.RECOVER_TIME = rc['recover_time']

        self.pose = {}                   # robot -> (x, y, yaw)
        self.trail = deque(maxlen=4000)  # list of (x, y, s)
        self.fidx = {f: 0 for f in self.FOLLOWERS}   # follower index on the trail
        self.obs_dist = {f: float('inf') for f in self.FOLLOWERS}  # /scan dist ahead
        self.blocked_t = {f: 0.0 for f in self.FOLLOWERS}  # time blocked by obstacle
        self.ss_active = {f: False for f in self.FOLLOWERS}  # side-stepping now
        self.ss_t = {f: 0.0 for f in self.FOLLOWERS}        # side-step timer
        self.ss_dir = {f: 1.0 for f in self.FOLLOWERS}      # which side (alternates)
        self.held = set()                # robots paused by the traffic manager
        self.leader_speed = 0.0
        self._last_leader_s = 0.0

        for name in [self.LEADER] + list(self.FOLLOWERS):
            self.create_subscription(
                Odometry, f'/{name}/ground_truth',
                self._make_pose_cb(name), 20)
        for name in self.FOLLOWERS:
            self.create_subscription(
                LaserScan, f'/{name}/scan',
                self._make_scan_cb(name), 10)
        self.create_subscription(String, '/fleet/hold', self._hold_cb, 10)

        self.cmd_pub = {f: self.create_publisher(Twist, f'/{f}/cmd_vel', 10)
                        for f in self.FOLLOWERS}
        self.path_pub = self.create_publisher(Path, '/convoy/leader_path', 1)

        self.dt = 0.05
        self.create_timer(self.dt, self._control)
        self.get_logger().info(
            f'Convoy up. Leader={self.LEADER}, followers={self.FOLLOWERS} '
            f'(gaps in m). Drive the leader with a Nav2 goal.')

    def _make_pose_cb(self, name):
        def cb(msg: Odometry):
            p = msg.pose.pose
            self.pose[name] = (p.position.x, p.position.y,
                               wb.yaw_from_quat(p.orientation.x, p.orientation.y,
                                                p.orientation.z, p.orientation.w))
            if name == self.LEADER:
                self._extend_trail(p.position.x, p.position.y)
        return cb

    def _make_scan_cb(self, name):
        def cb(msg: LaserScan):
            # Closest range within a +-OBSTACLE_CONE forward sector (0 = ahead).
            n = len(msg.ranges)
            if n == 0 or msg.angle_increment == 0.0:
                return
            closest = float('inf')
            for i, r in enumerate(msg.ranges):
                if not math.isfinite(r) or r < msg.range_min:
                    continue
                ang = wb.norm_angle(msg.angle_min + i * msg.angle_increment)
                if abs(ang) <= OBSTACLE_CONE and r < closest:
                    closest = r
            self.obs_dist[name] = closest
        return cb

    def _hold_cb(self, msg: String):
        try:
            self.held = set(json.loads(msg.data))
        except (ValueError, TypeError):
            self.held = set()

    def _extend_trail(self, x, y):
        if not self.trail:
            self.trail.append((x, y, 0.0))
            return
        lx, ly, ls = self.trail[-1]
        d = math.hypot(x - lx, y - ly)
        if d >= self.TRAIL_MIN_STEP:
            # The trail is a bounded deque: once it is full, the next append
            # evicts trail[0], shifting every element (and so every follower's
            # integer index) down by one. Decrement the indices to match, or
            # they would silently point at the wrong trail point after ~200 m.
            if len(self.trail) == self.trail.maxlen:
                for f in self.fidx:
                    self.fidx[f] = max(0, self.fidx[f] - 1)
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

    def _yield_if_leader_incoming(self, name, fx, fy, fyaw):
        """If the leader is heading toward this follower, pull perpendicular out
        of its path so it can pass, then return True (skip normal following)."""
        if self.LEADER not in self.pose:
            return False
        lx, ly, lyaw = self.pose[self.LEADER]
        d_lead = math.hypot(fx - lx, fy - ly)
        if d_lead > YIELD_DIST:
            return False
        # is this follower in front of the leader (leader driving at it)?
        rel = wb.norm_angle(math.atan2(fy - ly, fx - lx) - lyaw)
        if abs(rel) > 1.0:
            return False
        # step aside perpendicular to the leader's heading, to the side we're on
        side = 1.0 if rel >= 0 else -1.0
        ah = lyaw + side * (math.pi / 2.0)
        tx, ty = fx + YIELD_STEP * math.cos(ah), fy + YIELD_STEP * math.sin(ah)
        herr = wb.norm_angle(math.atan2(ty - fy, tx - fx) - fyaw)
        cmd = Twist()
        cmd.angular.z = max(-self.W_MAX, min(self.W_MAX, self.KP_YAW * herr))
        cmd.linear.x = 0.18 if abs(herr) < 1.0 else 0.0
        self.cmd_pub[name].publish(cmd)
        return True

    def _collision_block(self, name, fx, fy, fyaw):
        """True if another robot is within COLLISION_STOP in our forward half."""
        for other, pose in self.pose.items():
            if other == name:
                continue
            ox, oy, _ = pose
            if (math.hypot(ox - fx, oy - fy) < self.COLLISION_STOP and
                    abs(wb.norm_angle(math.atan2(oy - fy, ox - fx) - fyaw))
                    < math.pi / 2):
                return True
        return False

    def _run_sidestep(self, name, fx, fy, fyaw):
        """Steer aside to get around an obstacle the trail can't account for,
        then hand back to normal trail-following. Returns True while active."""
        self.ss_t[name] += self.dt
        od = self.obs_dist.get(name, float('inf'))
        cmd = Twist()
        cmd.angular.z = self.ss_dir[name] * self.W_MAX * 0.6
        # creep forward only when the way ahead and other robots are clear
        clear = od >= self.OBSTACLE_STOP and not self._collision_block(
            name, fx, fy, fyaw)
        cmd.linear.x = 0.12 if clear else 0.0
        self.cmd_pub[name].publish(cmd)
        if self.ss_t[name] >= self.RECOVER_TIME:
            self.ss_active[name] = False
            self.blocked_t[name] = 0.0
            return False
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

        for name, gap in self.FOLLOWERS.items():
            if name not in self.pose:
                continue
            fx, fy, fyaw = self.pose[name]

            # HOLD: the traffic manager has paused this robot -> stop and wait.
            if name in self.held:
                self.cmd_pub[name].publish(Twist())
                continue

            # GO-AROUND: keep running an in-progress obstacle side-step maneuver.
            if self.ss_active[name]:
                self._run_sidestep(name, fx, fy, fyaw)
                continue

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
            look_s = min(s_follower + self.LOOKAHEAD, s)
            tgt = wb.point_at_arclength(self.trail, look_s)
            if tgt is None:
                continue
            tx, ty = tgt

            # heading control (pure pursuit)
            heading_err = wb.norm_angle(math.atan2(ty - fy, tx - fx) - fyaw)
            w = max(-self.W_MAX, min(self.W_MAX, self.KP_YAW * heading_err))

            # longitudinal control: feed-forward leader speed + spacing error
            spacing_err = s_target - s_follower
            v = self.leader_speed + self.KP_LONG * spacing_err
            if abs(heading_err) > 0.8:
                v = min(v, 0.05)
            v = max(0.0, min(self.V_MAX, v))

            # SAFETY: never crowd the robot directly ahead.
            ahead = self.AHEAD.get(name)
            if ahead in self.pose:
                ax, ay, _ = self.pose[ahead]
                d_ahead = math.hypot(ax - fx, ay - fy)
                if d_ahead < self.MIN_GAP:
                    v = 0.0
                elif d_ahead < self.MIN_GAP + 0.4:
                    v = min(v, self.V_MAX * (d_ahead - self.MIN_GAP) / 0.4)

            # SAFETY 2: hard anti-collision against EVERY other robot, close range.
            if self._collision_block(name, fx, fy, fyaw):
                v = 0.0

            # SAFETY 3: dynamic obstacle on this robot's own /scan (something the
            # leader's recorded trail can't know about). Slow as we approach,
            # stop at OBSTACLE_STOP; if blocked too long, side-step around it.
            od = self.obs_dist.get(name, float('inf'))
            if od < self.OBSTACLE_STOP:
                v = 0.0
                self.blocked_t[name] += self.dt
                if self.blocked_t[name] > self.BLOCKED_TIME:
                    self.ss_active[name] = True
                    self.ss_t[name] = 0.0
                    self.ss_dir[name] *= -1.0    # alternate side each attempt
                    self.get_logger().warn(
                        f'{name} blocked by an obstacle - going around.')
            elif od < self.OBSTACLE_SLOW:
                v = min(v, self.V_MAX * (od - self.OBSTACLE_STOP) /
                        (self.OBSTACLE_SLOW - self.OBSTACLE_STOP))
                self.blocked_t[name] = 0.0
            else:
                self.blocked_t[name] = 0.0

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
        for pub in node.cmd_pub.values():
            pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
