#!/usr/bin/env python3
"""Publish map->odom for one robot from Gazebo ground truth.

Combines map->base_link (p3d plugin, /<ns>/ground_truth) with odom->base_link
(wheel odometry, /<ns>/odom) to get map->odom = (map->base) * (odom->base)^-1.
Used in place of AMCL in simulation so Nav2 runs on a pose that never diverges.
Set the namespace via the 'robot' parameter; the transform is published on
/<ns>/tf.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

import wb_common as wb


class GtLocalization(Node):
    def __init__(self):
        super().__init__('gt_localization')
        self.declare_parameter('robot', 'robot1')
        ns = self.get_parameter('robot').get_parameter_value().string_value
        self.ns = ns

        self.map_base = None    # (x, y, yaw)  from ground truth
        self.odom_base = None   # (x, y, yaw)  from wheel odometry

        self.create_subscription(Odometry, f'/{ns}/ground_truth',
                                 self._gt_cb, 20)
        self.create_subscription(Odometry, f'/{ns}/odom',
                                 self._odom_cb, 20)
        # publish straight onto the robot's namespaced tf topic
        self.tf_pub = self.create_publisher(TFMessage, f'/{ns}/tf', 20)
        self.create_timer(0.02, self._broadcast)   # 50 Hz
        self.get_logger().info(f'gt_localization publishing map->odom for {ns}')

    def _gt_cb(self, msg):
        p = msg.pose.pose
        self.map_base = (p.position.x, p.position.y,
                         wb.yaw_from_quat(p.orientation.x, p.orientation.y,
                                          p.orientation.z, p.orientation.w))

    def _odom_cb(self, msg):
        p = msg.pose.pose
        self.odom_base = (p.position.x, p.position.y,
                          wb.yaw_from_quat(p.orientation.x, p.orientation.y,
                                           p.orientation.z, p.orientation.w))

    def _broadcast(self):
        if self.map_base is None:
            return
        mx, my, myaw = self.map_base
        # Until wheel odometry is flowing, treat odom==base_link so the map->odom
        # transform (and thus the 'map' frame for RViz/costmaps) exists from the
        # moment ground truth arrives, right after spawn.
        ox, oy, oyaw = self.odom_base if self.odom_base is not None else (0.0, 0.0, 0.0)
        th = myaw - oyaw
        c, s = math.cos(th), math.sin(th)
        mo_x = mx - (c * ox - s * oy)
        mo_y = my - (s * ox + c * oy)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = mo_x
        t.transform.translation.y = mo_y
        t.transform.rotation.z = math.sin(th / 2.0)
        t.transform.rotation.w = math.cos(th / 2.0)
        self.tf_pub.publish(TFMessage(transforms=[t]))


def main():
    rclpy.init()
    node = GtLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
