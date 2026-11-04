#!/usr/bin/env python3
"""基于前沿(frontier)的自主探索建图节点。

和 auto_explore 的区别:
  auto_explore 是"反应式乱走"——撞墙就转, 不关心地图哪里还没扫到;
  本节点是"目标驱动"——每次从地图里找出 **空闲格与未知格的交界**(frontier),
  挑一个离自己最近的 frontier 作为目标开过去, 直到地图里再没有 frontier,
  此时认为整个场地已探索完毕, 自动停车退出(脚本随后保存地图)。

用法:
  ros2 run car_nav frontier_explore --ros-args -p use_sim_time:=true -p duration:=0.0
  参数:
    duration            最长运行秒数(仿真时间), 0 = 直到探索完成才退出
    linear_speed        前进速度, 默认 0.15 m/s
    angular_speed       原地转向角速度, 默认 0.9 rad/s
    goal_tolerance      到达 frontier 目标的判定距离, 默认 0.35 m
    front_stop          前方急停距离, 默认 0.32 m
    min_frontier_size   小于该格子数的 frontier 视为噪声, 默认 6
    replan_period       重新选目标的周期(秒), 默认 3.0
    give_up_time        目标超过该秒数仍未到达则拉黑, 默认 25.0
"""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)


def _min_range(ranges, a_min, a_max, angle_inc, angle_min):
    """取 [a_min, a_max] 扇区内最近的有效回波距离。"""
    if not ranges:
        return math.inf
    best = math.inf
    for i, r in enumerate(ranges):
        if not (r > 0.05 and r < 100.0):
            continue
        a = angle_min + i * angle_inc
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        if a_min <= a <= a_max:
            best = min(best, r)
    return best


class FrontierExplore(Node):

    def __init__(self):
        super().__init__('frontier_explore')

        self.declare_parameter('duration', 0.0)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.9)
        self.declare_parameter('goal_tolerance', 0.35)
        self.declare_parameter('front_stop', 0.32)
        self.declare_parameter('min_frontier_size', 4)
        self.declare_parameter('min_target_dist', 0.8)
        self.declare_parameter('replan_period', 3.0)
        self.declare_parameter('give_up_time', 25.0)
        self.declare_parameter('stuck_time', 8.0)
        self.declare_parameter('blacklist_ttl', 30.0)
        self.declare_parameter('stall_timeout', 150.0)
        self.declare_parameter('completion_checks', 4)

        p = self.get_parameter
        self.duration = float(p('duration').value)
        self.v = float(p('linear_speed').value)
        self.w = float(p('angular_speed').value)
        self.tol = float(p('goal_tolerance').value)
        self.front_stop = float(p('front_stop').value)
        self.min_size = int(p('min_frontier_size').value)
        self.min_target_dist = float(p('min_target_dist').value)
        self.replan_period = float(p('replan_period').value)
        self.give_up = float(p('give_up_time').value)
        self.stuck_time = float(p('stuck_time').value)
        self.blacklist_ttl = float(p('blacklist_ttl').value)
        self.stall_timeout = float(p('stall_timeout').value)
        self.completion_checks = int(p('completion_checks').value)

        map_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scan_qos = QoSProfile(depth=5,
                              reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(LaserScan, '/scan', self.on_scan, scan_qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map = None
        self.scan = None
        self.yaw = 0.0
        self.pose = None            # (x, y) in map frame

        self.target = None          # (x, y) in map frame
        self.target_time = None
        self.blacklist = []         # [(x, y, time_ns), ...] 已放弃的点
        self.last_dist = None       # 到目标的距离, 用于判断是否卡住
        self.last_progress_time = None
        self.turn_dir = None        # 避障转向方向, 锁定后不来回切换

        self.empty_checks = 0
        self.bl_log_time = None       # 上次打印"黑名单冷却中"的时刻
        self.turn_since = None        # 避障转向锁定的起始时刻
        self.start_time = None
        self.done = False
        self.best_unknown = 1.0         # 历史最小未知占比(仅用于打印)
        self.best_free = -1             # 历史最多自由格数(用于判断进展)
        self.best_free_time = None      # 上次刷新自由格数的时刻

        self.create_timer(1.0, self.plan_step)
        self.create_timer(0.1, self.control_step)

        self.get_logger().info(
            f'frontier_explore 启动: v={self.v} m/s, w={self.w} rad/s, '
            f'duration={self.duration}s (0=跑到探索完成)')

    # ---------------- 回调 ----------------
    def on_map(self, msg):
        self.map = msg

    def on_scan(self, msg):
        self.scan = msg

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z ** 2)

    # ---------------- 工具 ----------------
    def update_pose(self):
        """通过 TF 取 map 坐标系下的机器人位置。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            t = tf.transform.translation
            self.pose = (t.x, t.y)
            q = tf.transform.rotation
            self.yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z ** 2)
            return True
        except (LookupException, ConnectivityException,
                ExtrapolationException):
            return False

    def count_unknown(self):
        if self.map is None:
            return 1.0
        unknown = sum(1 for v in self.map.data if v < 0)
        return unknown / max(1, len(self.map.data))

    def count_free(self):
        """已探明可通过的格子数——只增不减, 是判断"有没有新进展"的可靠指标。

        不能用未知占比: slam_toolbox 扩展地图边界时未知格子会一起变多,
        占比反而上升, 会被误判成"停滞"。
        """
        if self.map is None:
            return 0
        return sum(1 for v in self.map.data if 0 <= v < 50)

    def find_frontiers(self):
        """返回 [(x, y, size)]（map 坐标系），已按距离升序排列。"""
        m = self.map
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        x0, y0 = m.info.origin.position.x, m.info.origin.position.y
        data = m.data

        free = bytearray(w * h)
        unknown = bytearray(w * h)
        for i in range(w * h):
            v = data[i]
            if v < 0:
                unknown[i] = 1
            elif v < 50:
                free[i] = 1

        frontier = []
        for y in range(1, h - 1):
            row = y * w
            for x in range(1, w - 1):
                i = row + x
                if not free[i]:
                    continue
                if (unknown[i - 1] or unknown[i + 1]
                        or unknown[i - w] or unknown[i + w]):
                    frontier.append(i)

        # BFS 聚类
        visited = set()
        clusters = []
        frontier_set = set(frontier)
        for start in frontier:
            if start in visited:
                continue
            q = deque([start])
            visited.add(start)
            cells = []
            while q:
                cur = q.popleft()
                cells.append(cur)
                cy, cx = divmod(cur, w)
                # 8 连通: 地图边缘噪点会打断 4 连通, 对角相连应算同一簇
                for ny in (cy - 1, cy, cy + 1):
                    for nx in (cx - 1, cx, cx + 1):
                        if (ny, nx) == (cy, cx):
                            continue
                        if 0 <= ny < h and 0 <= nx < w:
                            n = ny * w + nx
                            if n in frontier_set and n not in visited:
                                visited.add(n)
                                q.append(n)
            if len(cells) < self.min_size:
                continue
            sx = sum(c % w for c in cells) / len(cells)
            sy = sum(c // w for c in cells) / len(cells)
            wx = x0 + (sx + 0.5) * res
            wy = y0 + (sy + 0.5) * res
            clusters.append((wx, wy, len(cells)))

        px, py = self.pose
        clusters.sort(key=lambda c: math.hypot(c[0] - px, c[1] - py))
        return clusters

    def is_blacklisted(self, x, y):
        now_ns = self.get_clock().now().nanoseconds
        ttl_ns = self.blacklist_ttl * 1e9
        self.blacklist = [b for b in self.blacklist if now_ns - b[2] < ttl_ns]
        return any(math.hypot(x - b[0], y - b[1]) < 0.8 for b in self.blacklist)

    def give_up_target(self, reason):
        if self.target is None:
            return
        self.get_logger().warn(
            f'放弃目标 ({self.target[0]:.2f}, {self.target[1]:.2f}): {reason}')
        self.blacklist.append(
            (self.target[0], self.target[1], self.get_clock().now().nanoseconds))
        self.target = None
        self.last_dist = None

    # ---------------- 选目标 ----------------
    def plan_step(self):
        if self.done:
            return
        now = self.get_clock().now()

        if self.start_time is None:
            self.start_time = now
        if self.duration > 0 and \
                (now - self.start_time).nanoseconds * 1e-9 >= self.duration:
            self.get_logger().info('到达设定时长, 结束探索')
            self.finish()
            return

        if not self.update_pose():
            return
        if self.map is None:
            return

        # 停滞看门狗: 很久没有新探明的自由区域就收工, 避免死循环
        unknown_ratio = self.count_unknown()
        self.best_unknown = min(self.best_unknown, unknown_ratio)
        free_count = self.count_free()
        if free_count > self.best_free + 5:
            self.best_free = free_count
            self.best_free_time = now
        elif self.best_free_time is not None and \
                (now - self.best_free_time).nanoseconds * 1e-9 > self.stall_timeout:
            self.get_logger().info(
                f'{self.stall_timeout:.0f}s 没有探明新区域, 自由格停在 {free_count}, '
                f'未知占比 {unknown_ratio * 100:.2f}% -> 结束探索')
            self.finish()
            return

        # 目标超时未到达 -> 拉黑
        if self.target is not None and self.target_time is not None:
            if (now - self.target_time).nanoseconds * 1e-9 > self.give_up:
                self.give_up_target(f'{self.give_up:.0f}s 未到达')

        need_new = self.target is None
        if not need_new and self.target_time is not None:
            if (now - self.target_time).nanoseconds * 1e-9 >= self.replan_period:
                need_new = True

        clusters = self.find_frontiers()
        free_candidates = [c for c in clusters if not self.is_blacklisted(c[0], c[1])]
        # 优先选"离自己有一定距离"的 frontier, 避免在原地反复选同一个点
        candidates = [c for c in free_candidates
                      if math.hypot(c[0] - self.pose[0], c[1] - self.pose[1])
                      >= self.min_target_dist] or free_candidates

        # 注意: 只有"地图里真的一个 frontier 簇都没有"才算探索完成;
        # 剩余簇都在黑名单里只是暂时没目标, 等 TTL 过期后重试
        if not clusters:
            self.empty_checks += 1
            if self.empty_checks >= self.completion_checks:
                self.get_logger().info(
                    f'连续 {self.completion_checks} 次未发现 frontier, '
                    f'未知区域占比 {unknown_ratio * 100:.2f}% -> 探索完成')
                self.cmd_pub.publish(Twist())
                self.finish()
            return

        self.empty_checks = 0
        if not candidates:
            if self.target is None and (
                    self.bl_log_time is None or
                    (now - self.bl_log_time).nanoseconds * 1e-9 > 10.0):
                self.bl_log_time = now
                self.get_logger().info(
                    f'剩余 {len(clusters)} 个 frontier 都在黑名单冷却中, '
                    f'等冷却结束({self.blacklist_ttl:.0f}s)后重试...')
            return

        if need_new:
            new_target = (candidates[0][0], candidates[0][1])
            moved = self.target is None or math.hypot(
                new_target[0] - self.target[0], new_target[1] - self.target[1]) > 0.3
            self.target = new_target
            if moved:
                # 只有"真的换了目标"才重置计时器; 否则每 3s 的周期重规划会把
                # give_up/stuck 计时不断清零, 目标不可达时永远拉不黑、顶墙不放
                self.target_time = now
                self.last_dist = None
                self.last_progress_time = now
                self.get_logger().info(
                    f'新目标 ({self.target[0]:.2f}, {self.target[1]:.2f}), '
                    f'距离 {math.hypot(self.target[0] - self.pose[0], self.target[1] - self.pose[1]):.2f} m, '
                    f'frontier 簇数 {len(candidates)}, '
                    f'未知占比 {self.count_unknown() * 100:.1f}%')

    # ---------------- 控制 ----------------
    def control_step(self):
        if self.done or self.target is None or not self.update_pose():
            return

        dx = self.target[0] - self.pose[0]
        dy = self.target[1] - self.pose[1]
        dist = math.hypot(dx, dy)
        err = math.atan2(dy, dx) - self.yaw
        while err > math.pi:
            err -= 2 * math.pi
        while err < -math.pi:
            err += 2 * math.pi

        cmd = Twist()
        if dist < self.tol:
            # 到达该 frontier: 原地继续扫一圈, 然后把它拉黑, 换下一个点
            cmd.angular.z = self.w * 0.6
            self.cmd_pub.publish(cmd)
            self.give_up_target('已到达, 换下一个 frontier')
            return

        now = self.get_clock().now()
        if self.last_dist is None or self.last_dist - dist > 0.15:
            self.last_dist = dist
            self.last_progress_time = now
        elif self.last_progress_time is not None and \
                (now - self.last_progress_time).nanoseconds * 1e-9 > self.stuck_time:
            self.cmd_pub.publish(Twist())
            self.give_up_target(f'{self.stuck_time:.0f}s 没有靠近')
            return

        front = math.inf
        left = math.inf
        right = math.inf
        if self.scan is not None:
            front = _min_range(self.scan.ranges, -0.5, 0.5,
                               self.scan.angle_increment, self.scan.angle_min)
            left = _min_range(self.scan.ranges, 0.5, 1.4,
                              self.scan.angle_increment, self.scan.angle_min)
            right = _min_range(self.scan.ranges, -1.4, -0.5,
                               self.scan.angle_increment, self.scan.angle_min)

        if front < self.front_stop or (
                self.turn_dir is not None and front < self.front_stop * 2.0):
            # 前方太近: 锁定一个转向方向, 一直转到前方开阔为止(避免左右反复横跳)
            if self.turn_dir is None:
                self.turn_dir = 1 if left > right else -1
                self.turn_since = now
            cmd.angular.z = self.w * self.turn_dir
            self.target_time = now         # 避障期间不因超时/卡住被拉黑
            self.last_progress_time = now
            if self.turn_since is not None and \
                    (now - self.turn_since).nanoseconds * 1e-9 > 12.0:
                # 转了 12s 还转不出来(墙角卡死), 放弃这个目标
                self.turn_dir = None
                self.turn_since = None
                self.cmd_pub.publish(Twist())
                self.give_up_target('避障转向 12s 未脱困')
                return
        elif abs(err) > 0.6:
            self.turn_dir = None
            self.turn_since = None
            cmd.angular.z = self.w if err > 0 else -self.w
        else:
            self.turn_dir = None
            self.turn_since = None
            cmd.linear.x = self.v
            cmd.angular.z = max(-0.6, min(0.6, 1.5 * err))

        self.cmd_pub.publish(cmd)

    def finish(self):
        if self.done:
            return
        self.done = True
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplore()
    try:
        # 自己控制循环: 探索完成后立刻退出进程, 脚本才能继续保存地图
        while rclpy.ok() and not node.done:
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except rclpy.executors.ExternalShutdownException:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
