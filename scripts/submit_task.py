#!/usr/bin/env python3
"""CLI to submit a pick->drop task to the allocator.

Publishes one request on /fleet/task_request for task_allocator.py.

    ros2 run warehouse_bot_package submit_task.py pick_a drop_x
    ros2 run warehouse_bot_package submit_task.py pick_b drop_y --id job7
    ros2 run warehouse_bot_package submit_task.py --list   # list station names

Station names must exist in config/fleet.yaml.
"""

import argparse
import json
import sys

import rclpy
from std_msgs.msg import String

import wb_common as wb


def main():
    parser = argparse.ArgumentParser(description='Submit a warehouse task.')
    parser.add_argument('pickup', nargs='?', help='pickup station name')
    parser.add_argument('dropoff', nargs='?', help='dropoff station name')
    parser.add_argument('--id', dest='task_id', default=None, help='task id')
    parser.add_argument('--list', action='store_true',
                        help='list station names and exit')
    args = parser.parse_args()

    cfg = wb.load_fleet_config()
    stations = list(cfg['stations'])

    if args.list:
        print('Stations:', ', '.join(stations))
        return
    if not args.pickup or not args.dropoff:
        parser.error('pickup and dropoff are required (or use --list)')
    for st in (args.pickup, args.dropoff):
        if st not in stations:
            sys.exit(f'unknown station {st!r}; known: {", ".join(stations)}')

    task = {'pickup': args.pickup, 'dropoff': args.dropoff}
    if args.task_id:
        task['id'] = args.task_id

    rclpy.init()
    node = rclpy.create_node('submit_task')
    pub = node.create_publisher(String, '/fleet/task_request', 10)
    # let discovery connect before publishing the one-shot message
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() > 0:
            break
    pub.publish(String(data=json.dumps(task)))
    rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info(f'submitted task {task}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
