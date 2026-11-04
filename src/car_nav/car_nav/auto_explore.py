#!/usr/bin/env python3
"""自主探索建图节点（SLAM 演示用）。

反应式避障 + 周期性原地旋转扫描:
  - 订阅 /scan, 前方近障碍就转向更空的一侧, 否则慢速直行;
  - 每累计走 scan_interval 米, 原地转一整圈(帮 slam_toolbox 收集回环);
  - 边走边建图, duration 秒后自动停车退出。

慢速 + 旋转扫描显著降低里程计漂移, 多房间环境建图质量更好。

用法:
  ros2 run car_nav auto_explore --ros-args -p use_sim_time:=true -p duration:=420.0
  ros2 run car_nav auto_explore --ros-args -p use_sim_time:=true -p duration:=0.0   # 一直跑
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def _min_range(ranges, a_min, a_max):
    """取 [a_min, a_max]（弧度）扇区内最近的有效回波距离。"""
    if not ranges:
        return math.inf
    n = len(ranges)
    i0 = int(math.floor(a_min / (2 * math.pi) * n + 0.5))
    i1 = int(math.ceil(a_max / (2 * math.pi) * n + 0.5))
    best = math.inf
    for i in range(max(0, i0), min(n, i1 + 1)):
        r = ranges[i]
        if r < 0.05:            # 过滤 0 / inf / nan
            continue
        best = min(best, r)
    return best


class AutoExplore(Node):

    def __init__(self):
        super().__init__('auto_explore')
        self.declare_parameter('duration', 240.0)       # 秒(仿真时间), 0 = 不限
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('front_range', 0.45)     # 前方急停距离
        self.declare_parameter('side_range', 0.28)      # 侧向避让距离
        self.declare_parameter('scan_interval', 3.0)    # 每走几米原地转一圈
        self.declare_parameter('scan_angular_speed', 1.2)

        p = self.get_parameter
        self.duration = float(p('duration').value)
        self.v = float(p('linear_speed').value)
        self.w = float(p('angular_speed').value)
        self.front_range = float(p('front_range').value)
        self.side_range = float(p('side_range').value)
        self.scan_interval = float(p('scan_interval').value)
        self.scan_w = float(p('scan_angular_speed').value)

        self.ranges = []
        self.start_time = None
        self.traveled = 0.0
        self.last_x = None
        self.last_y = None
        self.scan_turned = 0.0            # 本次旋转扫描已转角度
        self.scanning = False
        self.last_yaw = None

        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'auto_explore 启动: v={self.v} m/s, w={self.w} rad/s, '
            f'每 {self.scan_interval} m 旋转扫描, duration={self.duration}s')

    def on_scan(self, msg):
        self.ranges = list(msg.ranges)

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = math.atan2(
            2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z),
            1.0 - 2.0 * msg.pose.pose.orientation.z ** 2)
        if self.last_x is not None:
            self.traveled += math.hypot(x - self.last_x, y - self.last_y)
            if self.last_yaw is not None:
                dyaw = (yaw - self.last_yaw + math.pi) % (2 * math.pi) - math.pi
                if self.scanning:
                    self.scan_turned += abs(dyaw)
        self.last_x, self.last_y, self.last_yaw = x, y, yaw

    def control_loop(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        cmd = Twist()
        if self.duration > 0 and \
                (self.get_clock().now() - self.start_time).nanoseconds * 1e-9 \
                >= self.duration:
            self.cmd_pub.publish(cmd)
            self.get_logger().info('探索时间到, 停车退出')
            rclpy.shutdown()
            return

        # ---- 旋转扫描模式: 转满一圈后回到行走模式 ----
        if self.scanning:
            if self.scan_turned >= 2 * math.pi - 0.15:
                self.scanning = False
                self.get_logger().info('旋转扫描完成, 继续探索 '
                                       f'(累计 {self.traveled:.0f} m)')
            else:
                cmd.angular.z = self.scan_w
                self.cmd_pub.publish(cmd)
                return
        elif self.traveled >= self.scan_interval:
            self.scanning = True
            self.scan_turned = 0.0
            self.traveled = 0.0
            self.get_logger().info('到达扫描点, 原地旋转一圈采集回环')
            cmd.angular.z = self.scan_w
            self.cmd_pub.publish(cmd)
            return

        if not self.ranges:
            self.cmd_pub.publish(cmd)
            return

        # TurtleBot3 激光: 角度 0 = 前方
        front = _min_range(self.ranges, -math.pi / 6, math.pi / 6)
        left = _min_range(self.ranges, math.pi / 6, math.pi / 2)
        right = _min_range(self.ranges, -math.pi / 2, -math.pi / 6)

        if front < self.front_range:
            # 前方受阻: 原地转向更空的一侧
            cmd.angular.z = self.w if left > right else -self.w
        elif left < self.side_range:
            cmd.linear.x = self.v
            cmd.angular.z = -self.w * 0.6
        elif right < self.side_range:
            cmd.linear.x = self.v
            cmd.angular.z = self.w * 0.6
        else:
            cmd.linear.x = self.v

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
