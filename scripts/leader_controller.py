#!/usr/bin/env python3
"""Drive the leader along a Nav2-planned path with pure pursuit.

Nav2's planner server still computes the global path (ComputePathToPose) on the
warehouse map; we drive along it with a pure-pursuit law rather than the stock
controller, which oscillates on this diff-drive robot at the 180-deg turn-around
heading. Goals arrive on /<leader>/goal_pose, pose on /<leader>/ground_truth,
and the path is republished on /<leader>/plan for RViz. If the leader stops
making progress it runs a recovery (rotate or back up, then replan) and gives up
after a few tries. Config: config/fleet.yaml.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose

import wb_common as wb


class LeaderController(Node):
    def __init__(self):
        super().__init__('leader_controller')

        cfg = wb.load_fleet_config()
        self.LEADER = cfg['convoy']['leader']
        self.OTHERS = [n for n in wb.robot_names(cfg) if n != self.LEADER]
        lc = cfg['leader']
        self.V_MAX = lc['v_max']
        self.W_MAX = lc['w_max']
        self.KP_YAW = lc['kp_yaw']
        self.LOOKAHEAD = lc['lookahead']
        self.GOAL_TOL = lc['goal_tol']
        self.REPLAN_PERIOD = lc['replan_period']
        self.STOP_DIST = lc['stop_dist']
        self.SLOW_DIST = lc['slow_dist']
        rc = cfg['recovery']
        self.STUCK_WINDOW = rc['stuck_window']
        self.STUCK_DIST = rc['stuck_dist']
        self.MAX_ATTEMPTS = rc['max_attempts']
        self.ROTATE_SPEED = rc['rotate_speed']
        self.BACKUP_SPEED = rc['backup_speed']
        self.RECOVER_TIME = rc['recover_time']
        self.BACKUP_CLEAR = rc['backup_clear']

        self.pose = None            # (x, y, yaw)
        self.others = {}            # robot -> (x, y)
        self.goal = None            # (x, y)
        self.path = []              # [(x, y), ...]
        self.idx = 0
        self._planning = False
        self._since_plan = 0.0
        # stuck-detection / recovery state
        self._progress_pos = None   # pose at the start of the stuck window
        self._stuck_t = 0.0         # time accumulated with little progress
        self._recovering = False
        self._recover_phase = None  # 'rotate' | 'backup'
        self._recover_t = 0.0
        self._attempts = 0
        self._plan_fails = 0

        self.create_subscription(Odometry, f'/{self.LEADER}/ground_truth',
                                 self._odom_cb, 20)
        for name in self.OTHERS:
            self.create_subscription(
                Odometry, f'/{name}/ground_truth',
                self._make_other_cb(name), 20)
        self.create_subscription(PoseStamped, f'/{self.LEADER}/goal_pose',
                                 self._goal_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, f'/{self.LEADER}/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, f'/{self.LEADER}/plan', 1)
        self.planner = ActionClient(self, ComputePathToPose,
                                    f'/{self.LEADER}/compute_path_to_pose')

        self.dt = 0.05
        self.create_timer(self.dt, self._control)
        self.get_logger().info(
            f'leader_controller up for {self.LEADER}; send a goal on '
            f'/{self.LEADER}/goal_pose (RViz "2D Goal Pose").')

    def _odom_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y,
                     wb.yaw_from_quat(p.orientation.x, p.orientation.y,
                                      p.orientation.z, p.orientation.w))

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
            if d > self.SLOW_DIST:
                continue
            bearing = wb.norm_angle(math.atan2(oy - y, ox - x) - yaw)
            if abs(bearing) > math.pi / 3.0:      # only a ~60deg cone ahead
                continue
            if d < self.STOP_DIST:
                return 0.0
            scale = min(scale, (d - self.STOP_DIST) /
                        (self.SLOW_DIST - self.STOP_DIST))
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
            self._plan_fails += 1
            self.get_logger().warn('planner rejected the goal; will retry.')
            return
        gh.get_result_async().add_done_callback(self._plan_done)

    def _plan_done(self, future):
        self._planning = False
        try:
            path = future.result().result.path
        except Exception:                      # planner aborted / no result
            path = None
        pts = [(p.pose.position.x, p.pose.position.y)
               for p in path.poses] if path else []
        if len(pts) >= 2:
            self.path = wb.smooth_polyline(pts)
            self.idx = 0
            self._plan_fails = 0
            self.path_pub.publish(path)
        else:
            self._plan_fails += 1
            if self._plan_fails in (1, 5, 15):
                self.get_logger().warn(
                    f'no path to goal yet (attempt {self._plan_fails}); '
                    f'retrying - goal may be blocked or in an obstacle.')

    def _rear_clear(self, x, y, yaw):
        """True if no other robot is close behind us (safe to reverse)."""
        for ox, oy in self.others.values():
            if math.hypot(ox - x, oy - y) > self.BACKUP_CLEAR:
                continue
            bearing = wb.norm_angle(math.atan2(oy - y, ox - x) - yaw)
            if abs(bearing) > 2.0:            # behind us (~115deg+ off the nose)
                return False
        return True

    def _start_recovery(self, x, y, yaw):
        self._recovering = True
        self._recover_t = 0.0
        self._attempts += 1
        self._recover_phase = 'backup' if self._rear_clear(x, y, yaw) else 'rotate'
        self.get_logger().warn(
            f'leader stuck - recovery {self._attempts}/{self.MAX_ATTEMPTS} '
            f'({self._recover_phase}).')

    def _run_recovery(self, x, y, yaw):
        """Execute the current recovery maneuver; return True while running."""
        self._recover_t += self.dt
        cmd = Twist()
        if self._recover_phase == 'backup':
            cmd.linear.x = -self.BACKUP_SPEED
            cmd.angular.z = self.ROTATE_SPEED * 0.5
        else:                                  # rotate in place to find a way out
            cmd.angular.z = self.ROTATE_SPEED
        self.cmd_pub.publish(cmd)
        if self._recover_t >= self.RECOVER_TIME:
            self._recovering = False
            self._stuck_t = 0.0
            self._progress_pos = (x, y)
            self._request_plan()               # replan after clearing
            return False
        return True

    def _update_stuck(self, x, y):
        """Accumulate stuck time; return True if we should start recovery."""
        if self._progress_pos is None:
            self._progress_pos = (x, y)
            self._stuck_t = 0.0
            return False
        moved = math.hypot(x - self._progress_pos[0], y - self._progress_pos[1])
        if moved > self.STUCK_DIST:
            self._progress_pos = (x, y)        # made progress -> reset
            self._stuck_t = 0.0
            return False
        self._stuck_t += self.dt
        if self._stuck_t >= self.STUCK_WINDOW:
            self._progress_pos = (x, y)
            return True
        return False

    def _control(self):
        self._since_plan += self.dt
        if self.pose is None or self.goal is None:
            return

        x, y, yaw = self.pose

        # mid-recovery: keep executing the maneuver, ignore normal control
        if self._recovering:
            self._run_recovery(x, y, yaw)
            return

        # reached goal?
        dgoal = math.hypot(self.goal[0] - x, self.goal[1] - y)
        if dgoal < self.GOAL_TOL:
            self.cmd_pub.publish(Twist())
            self.path = []
            self.goal = None      # idle until the next goal (stop re-publishing)
            self._attempts = 0
            self._progress_pos = None
            return

        # stuck? (trying to reach a goal but not moving) -> recover or give up
        if self._update_stuck(x, y):
            if self._attempts >= self.MAX_ATTEMPTS:
                self.get_logger().error(
                    f'giving up goal {self.goal} after {self._attempts} '
                    f'recovery attempts; send a new goal.')
                self.cmd_pub.publish(Twist())
                self.goal = None
                self.path = []
                self._attempts = 0
                return
            self._start_recovery(x, y, yaw)
            return

        # replan faster while blocked; otherwise on the normal period
        period = 1.0 if self._stuck_t > 1.0 else self.REPLAN_PERIOD
        if not self.path or self._since_plan > period:
            self._request_plan()
        if not self.path:
            return
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
            if acc >= self.LOOKAHEAD:
                tx, ty = self.path[j + 1]
                break

        heading_err = wb.norm_angle(math.atan2(ty - y, tx - x) - yaw)
        w = max(-self.W_MAX, min(self.W_MAX, self.KP_YAW * heading_err))
        v = self.V_MAX * max(0.0, math.cos(heading_err))   # slow down to turn
        if abs(heading_err) > 1.0:                         # turn in place if way off
            v = 0.0
        v = min(v, self.V_MAX * dgoal / 0.5)               # ease in near the goal
        v *= self._avoid_scale(x, y, yaw)                  # don't hit other robots

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
