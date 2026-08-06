#!/usr/bin/env python3
"""Warehouse task dispatch for the fleet.

Turns "move a load from station A to station B" into Nav2 goals. Stations come
from config/fleet.yaml; tasks arrive on /fleet/task_request as JSON, e.g.
{"id": "t1", "pickup": "pick_a", "dropoff": "drop_x"} (see submit_task.py).
Fleet poses come from /fleet/status. Each queued task is given to the nearest
idle, un-held robot, which drives pickup then dropoff via its navigate_to_pose
action and goes idle again. Task state is published on /fleet/tasks. Needs the
full per-robot Nav2 stacks (bringup.launch.py).
"""

import json
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import wb_common as wb


class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')

        cfg = wb.load_fleet_config()
        self.robots = wb.robot_names(cfg)
        self.stations = cfg['stations']

        self.poses = {}            # robot -> (x, y, yaw) from /fleet/status
        self.held = set()          # robots paused by the traffic manager
        self.busy = {}             # robot -> task dict it is currently running
        self.queue = deque()       # pending task dicts
        self._seq = 0              # fallback id counter

        self.nav_clients = {r: ActionClient(self, NavigateToPose,
                                        f'/{r}/navigate_to_pose')
                        for r in self.robots}

        self.create_subscription(String, '/fleet/status', self._status_cb, 10)
        self.create_subscription(String, '/fleet/hold', self._hold_cb, 10)
        self.create_subscription(String, '/fleet/task_request',
                                 self._request_cb, 10)
        self.tasks_pub = self.create_publisher(String, '/fleet/tasks', 10)

        self.create_timer(1.0, self._assign)
        self.get_logger().info(
            f'Task allocator up. Robots={self.robots}, '
            f'stations={list(self.stations)}. '
            f'Send tasks on /fleet/task_request.')

    # ----------------------------------------------------------- callbacks
    def _status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        for name, p in data.items():
            self.poses[name] = (p['x'], p['y'], p.get('yaw', 0.0))

    def _hold_cb(self, msg: String):
        try:
            self.held = set(json.loads(msg.data))
        except (ValueError, TypeError):
            self.held = set()

    def _request_cb(self, msg: String):
        try:
            task = json.loads(msg.data)
        except ValueError:
            self.get_logger().error(f'bad task request (not JSON): {msg.data!r}')
            return
        pickup, dropoff = task.get('pickup'), task.get('dropoff')
        if pickup not in self.stations or dropoff not in self.stations:
            self.get_logger().error(
                f'task rejected: unknown station(s) {pickup!r}/{dropoff!r}. '
                f'Known: {list(self.stations)}')
            return
        if 'id' not in task:
            self._seq += 1
            task['id'] = f't{self._seq}'
        self.queue.append({'id': task['id'], 'pickup': pickup,
                           'dropoff': dropoff})
        self.get_logger().info(
            f'queued task {task["id"]}: {pickup} -> {dropoff} '
            f'({len(self.queue)} pending)')
        self._publish_tasks()

    # ------------------------------------------------------------- dispatch
    def _idle_robots(self):
        return [r for r in self.robots
                if r not in self.busy and r not in self.held
                and r in self.poses]

    def _assign(self):
        self._publish_tasks()
        while self.queue:
            cands = {r: self.poses[r] for r in self._idle_robots()}
            if not cands:
                return
            task = self.queue[0]
            pick = self.stations[task['pickup']]
            robot = wb.nearest(cands, (pick['x'], pick['y']))
            if robot is None:
                return
            if not self.nav_clients[robot].server_is_ready():
                # Nav2 not up yet for this robot; try again next tick.
                return
            self.queue.popleft()
            self.busy[robot] = task
            self.get_logger().info(
                f'assign task {task["id"]} to {robot}: '
                f'{task["pickup"]} -> {task["dropoff"]}')
            self._drive(robot, task['pickup'], 'to_pickup')

    def _drive(self, robot, station_key, phase):
        st = self.stations[station_key]
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = float(st['x'])
        p.pose.position.y = float(st['y'])
        z, w = wb.quat_from_yaw(float(st.get('yaw', 0.0)))
        p.pose.orientation.z = z
        p.pose.orientation.w = w
        goal.pose = p
        send = self.nav_clients[robot].send_goal_async(goal)
        send.add_done_callback(
            lambda fut, r=robot, ph=phase: self._goal_response(fut, r, ph))

    def _goal_response(self, future, robot, phase):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn(f'{robot}: goal rejected ({phase}); requeueing.')
            self._release(robot, requeue=True)
            return
        gh.get_result_async().add_done_callback(
            lambda fut, r=robot, ph=phase: self._goal_done(fut, r, ph))

    def _goal_done(self, future, robot, phase):
        task = self.busy.get(robot)
        if task is None:
            return
        if phase == 'to_pickup':
            self.get_logger().info(
                f'{robot}: reached pickup {task["pickup"]} -> driving to dropoff.')
            self._drive(robot, task['dropoff'], 'to_dropoff')
        else:
            self.get_logger().info(
                f'{robot}: task {task["id"]} complete at {task["dropoff"]}.')
            self._release(robot)

    def _release(self, robot, requeue=False):
        task = self.busy.pop(robot, None)
        if requeue and task is not None:
            self.queue.appendleft(task)
        self._publish_tasks()

    def _publish_tasks(self):
        state = {
            'pending': [t['id'] for t in self.queue],
            'active': {r: t['id'] for r, t in self.busy.items()},
            'held': sorted(self.held),
        }
        self.tasks_pub.publish(String(data=json.dumps(state)))


def main():
    rclpy.init()
    node = TaskAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
