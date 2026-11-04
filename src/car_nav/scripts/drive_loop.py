#!/usr/bin/env python3
"""截图用: 让 TurtleBot3 在障碍场里按固定路线跑一段, 把 SLAM 地图画大。

只发 /cmd_vel, 不做避障, 路线是按 obstacle_field.world 的空旷走廊手工编排的。
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

ROUTE = [
    ('forward', 3.0),   # -4.5,-4.5 向 +x 走
    ('turn_left', 1.0),
    ('forward', 3.0),
    ('turn_right', 1.0),
    ('forward', 3.0),
    ('turn_left', 1.0),
    ('forward', 2.5),
    ('turn_left', 1.2),
    ('forward', 2.5),
]

LIN = 0.18
ANG = 0.9


def main():
    rclpy.init()
    node = Node('screenshot_driver')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    node.get_logger().info('开始按预设路线行驶')
    msg = Twist()
    for action, dur in ROUTE:
        msg.linear.x = LIN if action == 'forward' else 0.0
        msg.angular.z = ANG if action == 'turn_left' else (
            -ANG if action == 'turn_right' else 0.0)
        t_end = time.time() + dur
        while time.time() < t_end and rclpy.ok():
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.05)
    pub.publish(Twist())
    node.get_logger().info('路线结束')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()