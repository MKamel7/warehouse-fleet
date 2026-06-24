#!/usr/bin/env python3
"""
leader_controller  -  drive the leader along a Nav2-planned path
================================================================

The leader still gets a REAL globally-planned path from Nav2's planner server
(A*/NavFn on the warehouse map), but is driven along it by a smooth pure-pursuit
law instead of the stock controller. The stock RPP/rotation-shim controller
oscillates on this diff-drive robot at the 180-deg "turn-around" heading; the
pure-pursuit law (the same one the convoy followers use) arcs onto the path
cleanly and never sticks at that singularity.

Flow:
  goal (/<leader>/goal_pose, e.g. RViz "2D Goal Pose")
    -> ComputePathToPose action (planner)  -> path
    -> pure-pursuit along the path          -> /<leader>/cmd_vel
Pose comes from /<leader>/ground_truth. Path is republished on
/<leader>/plan for RViz.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose

LEADER = 'robot1'
OTHERS = ['robot2', 'robot3']    # robots to avoid hitting
V_MAX = 0.26
W_MAX = 1.2
KP_YAW = 1.6
LOOKAHEAD = 0.5
GOAL_TOL = 0.25
REPLAN_PERIOD = 2.0       # seconds between replans while driving
STOP_DIST = 0.45          # hard stop if another robot is this close ahead
SLOW_DIST = 0.85          # start slowing at this distance


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _smooth(pts, passes=3, alpha=0.5):
    """Moving-average smoothing of a polyline; keeps endpoints fixed."""
    p = list(pts)
    for _ in range(passes):
        q = list(p)
        for i in range(1, len(p) - 1):
            q[i] = (p[i][0] + alpha * (p[i - 1][0] + p[i + 1][0] - 2 * p[i][0]),
                    p[i][1] + alpha * (p[i - 1][1] + p[i + 1][1] - 2 * p[i][1]))
        p = q
    return p


class LeaderController(Node):
    def __init__(self):
        super().__init__('leader_controller')
        self.pose = None            # (x, y, yaw)
        self.others = {}            # robot -> (x, y)
        self.goal = None            # (x, y)
        self.path = []              # [(x, y), ...]
        self.idx = 0
        self._planning = False
        self._since_plan = 0.0

        self.create_subscription(Odometry, f'/{LEADER}/ground_truth',
                                 self._odom_cb, 20)
        for name in OTHERS:
            self.create_subscription(
                Odometry, f'/{name}/ground_truth',
                self._make_other_cb(name), 20)
        self.create_subscription(PoseStamped, f'/{LEADER}/goal_pose',
                                 self._goal_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, f'/{LEADER}/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, f'/{LEADER}/plan', 1)
        self.planner = ActionClient(self, ComputePathToPose,
                                    f'/{LEADER}/compute_path_to_pose')

        self.dt = 0.05
        self.create_timer(self.dt, self._control)
        self.get_logger().info(
            f'leader_controller up for {LEADER}; send a goal on '
            f'/{LEADER}/goal_pose (RViz "2D Goal Pose").')

    def _odom_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _make_other_cb(self, name):
        def cb(msg):
            self.others[name] = (msg.pose.pose.position.x,
                                 msg.pose.pose.position.y)
        return cb

    def _avoid_scale(self, x, y, yaw):
        """Return a speed scale in [0,1]: 0 = stop, 1 = clear, based on the
        nearest other robot that lies ahead of us."""
        scale = 1.0
        for ox, oy in self.others.values():
            d = math.hypot(ox - x, oy - y)
            if d > SLOW_DIST:
                continue
            bearing = norm(math.atan2(oy - y, ox - x) - yaw)
            if abs(bearing) > math.pi / 3.0:      # only a ~60deg cone ahead
                continue
            if d < STOP_DIST:
                return 0.0
            scale = min(scale, (d - STOP_DIST) / (SLOW_DIST - STOP_DIST))
        return scale

    def _goal_cb(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'new goal {self.goal}, planning...')
        self._request_plan()

    def _request_plan(self):
        if self.goal is None or self.pose is None or self._planning:
            return
        if not self.planner.server_is_ready():
            return
        g = ComputePathToPose.Goal()
        g.goal.header.frame_id = 'map'
        g.goal.pose.position.x = float(self.goal[0])
        g.goal.pose.position.y = float(self.goal[1])
        g.goal.pose.orientation.w = 1.0
        g.use_start = False
        self._planning = True
        self._since_plan = 0.0
        self.planner.send_goal_async(g).add_done_callback(self._plan_resp)

    def _plan_resp(self, future):
        gh = future.result()
        if not gh.accepted:
            self._planning = False
            return
        gh.get_result_async().add_done_callback(self._plan_done)

    def _plan_done(self, future):
        self._planning = False
        path = future.result().result.path
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        if len(pts) >= 2:
            self.path = _smooth(pts)
            self.idx = 0
            self.path_pub.publish(path)

    def _control(self):
        self._since_plan += self.dt
        if self.pose is None or self.goal is None:
            return

        # reached goal?
        dgoal = math.hypot(self.goal[0] - self.pose[0], self.goal[1] - self.pose[1])
        if dgoal < GOAL_TOL:
            self.cmd_pub.publish(Twist())
            self.path = []
            return

        # periodic replans keep the path fresh
        if not self.path or self._since_plan > REPLAN_PERIOD:
            self._request_plan()
        if not self.path:
            return

        x, y, yaw = self.pose
        # advance index to nearest path point
        best, bestd = self.idx, float('inf')
        for j in range(self.idx, len(self.path)):
            d = (x - self.path[j][0]) ** 2 + (y - self.path[j][1]) ** 2
            if d < bestd:
                bestd, best = d, j
        self.idx = best

        # look-ahead point
        tx, ty = self.path[-1]
        acc = 0.0
        for j in range(self.idx, len(self.path) - 1):
            acc += math.hypot(self.path[j + 1][0] - self.path[j][0],
                              self.path[j + 1][1] - self.path[j][1])
            if acc >= LOOKAHEAD:
                tx, ty = self.path[j + 1]
                break

        heading_err = norm(math.atan2(ty - y, tx - x) - yaw)
        w = max(-W_MAX, min(W_MAX, KP_YAW * heading_err))
        v = V_MAX * max(0.0, math.cos(heading_err))   # slow down to turn
        if abs(heading_err) > 1.0:                     # turn in place if way off
            v = 0.0
        v = min(v, V_MAX * dgoal / 0.5)                # ease in near the goal
        v *= self._avoid_scale(x, y, yaw)              # don't hit other robots

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = LeaderController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
