#!/usr/bin/env python3
"""航点巡逻任务节点（图3 效果）。

通过 nav2_simple_commander 依次导航到预设航点, 终端实时打印任务进度:
    === 任务开始 ===
    >>> 前往航点 1/5: (x, y)
        航点 1/5 剩余距离 1.98 m
    ✓ 航点 1/5 到达
    === 任务完成: 5/5 个航点成功, 总耗时 xx 秒 ===

同时在 RViz 发布航点标记 (/mission/waypoints): 橙色编号圆球 + 文字标签。

用法:
  ros2 run car_nav mission_runner
  ros2 run car_nav mission_runner --ros-args -p loop:=true
  ros2 run car_nav mission_runner --ros-args \
      -p waypoint_x:="[-1.2, 1.5]" -p waypoint_y:="[0.5, -1.0]"
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class MissionRunner(Node):

    def __init__(self):
        super().__init__('mission_runner')
        # 房屋地图巡逻航点默认值 (建图完成后按实际地图调整)
        self.declare_parameter('waypoint_x', [-1.0])
        self.declare_parameter('waypoint_y', [-1.0])
        self.declare_parameter('waypoint_yaw', [0.0])
        self.declare_parameter('loop', False)
        self.declare_parameter('log_period', 3.0)
        # 机器人出生点: BasicNavigator.waitUntilNav2Active() 会主动发布一次
        # /initialpose(它自己的 initial_pose 默认是 (0,0,0) 且四元数全 0),
        # 不先把真值喂给它, AMCL 会被拉到地图原点, 之后全局路径就"不在局部
        # 代价地图里"、DWB 报 No valid trajectories。
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.marker_pub = self.create_publisher(MarkerArray, '/mission/waypoints', 10)
        self.navigator = BasicNavigator()

    # ------------------------------------------------------------------ 航点
    def get_waypoints(self):
        p = self.get_parameter
        xs = [float(v) for v in p('waypoint_x').value]
        ys = [float(v) for v in p('waypoint_y').value]
        yaws = [float(v) for v in p('waypoint_yaw').value]
        while len(yaws) < len(xs):
            yaws.append(0.0)
        return list(zip(xs, ys, yaws))

    def build_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def publish_waypoint_markers(self, wps):
        msg = MarkerArray()
        for i, (x, y, _) in enumerate(wps):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'mission_waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.22
            m.color.r = 0.98
            m.color.g = 0.55
            m.color.b = 0.05
            m.color.a = 0.95
            msg.markers.append(m)

            t = Marker()
            t.header = m.header
            t.ns = 'mission_labels'
            t.id = 100 + i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(x)
            t.pose.position.y = float(y)
            t.pose.position.z = 0.35
            t.scale.z = 0.28
            t.color.r = t.color.g = t.color.b = 1.0
            t.color.a = 1.0
            t.text = f'WP{i + 1}'
            msg.markers.append(t)
        self.marker_pub.publish(msg)

    # ------------------------------------------------------------------ 主流程
    def run(self):
        wps = self.get_waypoints()
        loop = bool(self.get_parameter('loop').value)
        log_period = float(self.get_parameter('log_period').value)

        self.get_logger().info('=== 等待 Nav2 就绪 ===')
        # 先把出生点交给 BasicNavigator, 否则它在 _waitForInitialPose 里会发布
        # 全零初始位姿, 把 AMCL 拽到地图原点, 整个导航随即失效
        self.navigator.setInitialPose(self.build_pose(
            float(self.get_parameter('initial_x').value),
            float(self.get_parameter('initial_y').value),
            float(self.get_parameter('initial_yaw').value)))
        self.navigator.waitUntilNav2Active()
        self.get_logger().info(
            f'=== 任务开始: {len(wps)} 个航点'
            f'{" (循环模式)" if loop else ""} ===')
        self.publish_waypoint_markers(wps)

        t0 = time.time()
        n_ok = 0
        n_total = 0
        try:
            while rclpy.ok():
                for i, (x, y, yaw) in enumerate(wps):
                    if not rclpy.ok():
                        return
                    n_total += 1
                    self.get_logger().info(
                        f'>>> 前往航点 {i + 1}/{len(wps)}: ({x:.2f}, {y:.2f})')
                    self.navigator.goToPose(self.build_pose(x, y, yaw))

                    last_log = 0.0
                    while not self.navigator.isTaskComplete():
                        fb = self.navigator.getFeedback()
                        if fb is not None and time.time() - last_log >= log_period:
                            self.get_logger().info(
                                f'    航点 {i + 1}/{len(wps)} '
                                f'剩余距离 {fb.distance_remaining:.2f} m')
                            last_log = time.time()

                    result = self.navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        n_ok += 1
                        self.get_logger().info(f'✓ 航点 {i + 1}/{len(wps)} 到达')
                    elif result == TaskResult.CANCELED:
                        self.get_logger().warn(f'航点 {i + 1} 被取消, 终止任务')
                        return
                    else:
                        self.get_logger().warn(f'✗ 航点 {i + 1} 失败, 跳过')
                if not loop:
                    break
        finally:
            self.get_logger().info(
                f'=== 任务完成: {n_ok}/{n_total} 个航点成功, '
                f'总耗时 {time.time() - t0:.0f} 秒 ===')

    def destroy_node(self):
        try:
            self.navigator.destroy_node()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionRunner()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
