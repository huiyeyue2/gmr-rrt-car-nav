#!/usr/bin/env python3
"""向 Nav2 下发 NavigateToPose 目标, 便于无 RViz 时脚本化导航。

用法:
  ros2 run car_nav goal_client --ros-args -p goal_x:=1.0 -p goal_y:=-1.0 -p goal_yaw:=1.57
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalClient(Node):

    def __init__(self):
        super().__init__('goal_client')
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send(self):
        x = self.get_parameter('goal_x').value
        y = self.get_parameter('goal_y').value
        yaw = self.get_parameter('goal_yaw').value

        self.get_logger().info('等待 navigate_to_pose action server ...')
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('navigate_to_pose 不可用, 放弃')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        self.get_logger().info(f'下发目标 ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
        future = self.client.send_goal_async(
            goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, future)
        if not rclpy.ok() or future.result() is None:
            self.get_logger().error('发送目标失败或已中止')
            return

        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('目标被拒绝')
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if not rclpy.ok() or result_future.result() is None:
            self.get_logger().error('未拿到导航结果')
            return
        self.get_logger().info(
            f'导航结束, action status = {result_future.result().status}')

    def on_feedback(self, feedback):
        d = feedback.feedback.distance_remaining
        self.get_logger().info(f'剩余距离 {d:.2f} m', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = GoalClient()
    try:
        node.send()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
