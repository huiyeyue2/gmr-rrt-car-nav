#!/usr/bin/env python3
"""启动后自动给 AMCL 设置初始位姿, 避免手点 2D Pose Estimate。

发布后收到 /amcl_pose 即认为 AMCL 已收敛, 自行退出;
最多重发 max_attempts 次(默认 8s), 保证在航点任务启动(launch 里 35s)前就停手,
否则任务开始后还在重置粒子滤波, 会把机器人从第一个航点拽走。
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.declare_parameter('x', -2.0)
        self.declare_parameter('y', -0.5)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('max_attempts', 8)
        self.declare_parameter('confirm_poses', 2)   # 发布后收到几条 /amcl_pose 判定收敛

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)

        self.attempts = 0
        self.received = 0
        self.done = False
        self.create_timer(1.0, self.tick)

    def tick(self):
        if self.received >= int(self.get_parameter('confirm_poses').value):
            self.get_logger().info('AMCL 已输出位姿, 停止重发初始位姿')
            self.done = True
            return
        if self.attempts >= self.get_parameter('max_attempts').value:
            self.get_logger().warn('达到最大尝试次数, 退出')
            self.done = True
            return

        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.pub.publish(msg)

        self.attempts += 1
        self.get_logger().info(
            f'发布初始位姿 ({x:.2f}, {y:.2f}, yaw={yaw:.2f}) '
            f'第 {self.attempts} 次')

    def on_amcl(self, msg):
        # 只统计"发出 /initialpose 之后"的位姿, 避免把 AMCL 旧的默认估计当成收敛
        if self.attempts > 0:
            self.received += 1


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
