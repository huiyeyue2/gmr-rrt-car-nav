#!/usr/bin/env python3
"""基于前沿(frontier)的自主探索建图节点。

和 auto_explore 的区别:
  auto_explore 是"反应式乱走"——撞墙就转, 不关心地图哪里还没扫到;
  本节点是"目标驱动"——每次从地图里找出 **空闲格与未知格的交界**(frontier),
  挑一个效用最高(信息增益/距离)的 frontier 作为目标开过去, 直到地图覆盖率
  达到设定阈值(默认 95%)或已无 frontier, 自动停车退出(脚本随后保存地图)。

主要改进:
  - 避障不只原地转, 改为"沿障碍物切线低速前行 + 锁定转向方向", 防止反复横跳
  - 到达目标后原地扫 2.5s 再拉黑, 确保 SLAM 把柱子/墙角附近画清楚
  - 效用评分对黑名单 1.0m 内的 frontier 簇乘 0.3 衰减, 不再围着同一柱反复绕
  - 覆盖率连续 5 次 ≥ 95% 才判定完成, 扛住 SLAM 扩边界导致的占比波动

用法:
  ros2 run car_nav frontier_explore --ros-args -p use_sim_time:=true -p duration:=0.0
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
        self.declare_parameter('target_coverage', 0.95)
        self.declare_parameter('coverage_checks', 5)
        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 0.9)
        self.declare_parameter('goal_tolerance', 0.55)
        self.declare_parameter('front_stop', 0.45)
        self.declare_parameter('min_frontier_size', 4)
        self.declare_parameter('min_target_dist', 0.8)
        self.declare_parameter('replan_period', 5.0)
        self.declare_parameter('give_up_time', 20.0)
        self.declare_parameter('stuck_time', 8.0)
        self.declare_parameter('blacklist_ttl', 30.0)
        self.declare_parameter('stall_timeout', 150.0)
        self.declare_parameter('completion_checks', 4)
        self.declare_parameter('arrive_pause', 2.5)
        self.declare_parameter('avoid_radius', 1.0)

        p = self.get_parameter
        self.duration = float(p('duration').value)
        self.target_coverage = float(p('target_coverage').value)
        self.coverage_checks = int(p('coverage_checks').value)
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
        self.arrive_pause = float(p('arrive_pause').value)
        self.avoid_radius = float(p('avoid_radius').value)

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
        self.pose = None

        self.target = None
        self.target_time = None
        self.blacklist = []
        self.last_dist = None
        self.last_progress_time = None
        self.turn_dir = None

        self.empty_checks = 0
        self.bl_log_time = None
        self.turn_since = None
        self.start_time = None
        self.done = False
        self.best_unknown = 1.0
        self.best_free = -1
        self.best_free_time = None
        self.coverage_hits = 0
        self.arrive_deadline = None     # 到达后原地扫描截止时刻, None=不在扫描

        self.create_timer(1.0, self.plan_step)
        self.create_timer(0.1, self.control_step)

        self.get_logger().info(
            f'frontier_explore 启动: v={self.v} m/s, w={self.w} rad/s, '
            f'tol={self.tol}m, front_stop={self.front_stop}m, '
            f'目标覆盖率={self.target_coverage * 100:.0f}%')

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
        if self.map is None:
            return 0
        return sum(1 for v in self.map.data if 0 <= v < 50)

    def find_frontiers(self):
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
        return any(math.hypot(x - b[0], y - b[1]) < self.avoid_radius
                   for b in self.blacklist)

    def _in_blacklist_zone(self, x, y):
        """该点是否落在任何黑名单点的衰减半径内（影响效用评分）。"""
        return any(math.hypot(x - b[0], y - b[1]) < self.avoid_radius
                   for b in self.blacklist)

    def _information_gain_at(self, wx, wy, radius):
        """以世界坐标 (wx, wy) 为圆心、radius 为半径, 统计圆内未知格数量。

        参考 rrt_exploration 的经典实现: informationGain(mapData, point, r)。
        比直接用 frontier 簇格子数更准确, 因为它考虑了"站在该点激光雷达实际能扫到多少新区域"。
        柱子/厚墙附近的 frontier 虽然簇格子多, 但激光被挡住, 实际 gain 会很低。
        """
        m = self.map
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        x0, y0 = m.info.origin.position.x, m.info.origin.position.y

        cx = int((wx - x0) / res)
        cy = int((wy - y0) / res)
        rr = int(radius / res)
        count = 0
        # 局部窗口遍历, 避免扫描整张地图
        for dy in range(-rr, rr + 1):
            ny = cy + dy
            if ny < 0 or ny >= h:
                continue
            row = ny * w
            for dx in range(-rr, rr + 1):
                nx = cx + dx
                if nx < 0 or nx >= w:
                    continue
                if dx * dx + dy * dy > rr * rr:
                    continue
                v = m.data[row + nx]
                if v < 0:  # 未知格
                    count += 1
        return count

    def give_up_target(self, reason):
        if self.target is None:
            return
        self.get_logger().warn(
            f'放弃目标 ({self.target[0]:.2f}, {self.target[1]:.2f}): {reason}')
        self.blacklist.append(
            (self.target[0], self.target[1], self.get_clock().now().nanoseconds))
        self.target = None
        self.last_dist = None
        self.arrive_deadline = None

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

        unknown_ratio = self.count_unknown()
        self.best_unknown = min(self.best_unknown, unknown_ratio)
        coverage = 1.0 - unknown_ratio
        if coverage >= self.target_coverage:
            self.coverage_hits += 1
            self.get_logger().info(
                f'覆盖率 {coverage * 100:.2f}% >= '
                f'{self.target_coverage * 100:.2f}% '
                f'({self.coverage_hits}/{self.coverage_checks})')
            if self.coverage_hits >= self.coverage_checks:
                self.get_logger().info(
                    f'覆盖率连续 {self.coverage_checks} 次达标, 未知仅剩 '
                    f'{unknown_ratio * 100:.2f}% -> 探索完成')
                self.finish()
                return
        else:
            self.coverage_hits = 0

        free_count = self.count_free()
        if free_count > self.best_free + 5:
            self.best_free = free_count
            self.best_free_time = now
        elif self.best_free_time is not None and \
                (now - self.best_free_time).nanoseconds * 1e-9 > self.stall_timeout:
            self.get_logger().info(
                f'{self.stall_timeout:.0f}s 无新自由格 ({free_count}), '
                f'未知占比 {unknown_ratio * 100:.2f}% -> 结束探索')
            self.finish()
            return

        if self.target is not None and self.target_time is not None:
            if (now - self.target_time).nanoseconds * 1e-9 > self.give_up:
                self.give_up_target(f'{self.give_up:.0f}s 未到达')

        # 到达后原地扫描中 —— 不选新目标, 等倒计时结束再拉黑
        if self.arrive_deadline is not None:
            if now.nanoseconds >= self.arrive_deadline:
                self.get_logger().info(
                    '到达目标后扫描完成, 拉黑并寻找下一个 frontier')
                self.give_up_target('到达, 扫描完成换下一个')
            else:
                return

        need_new = self.target is None
        if not need_new and self.target_time is not None:
            if (now - self.target_time).nanoseconds * 1e-9 >= self.replan_period:
                need_new = True

        clusters = self.find_frontiers()
        free_candidates = [c for c in clusters if not self.is_blacklisted(c[0], c[1])]
        candidates = [c for c in free_candidates
                      if math.hypot(c[0] - self.pose[0], c[1] - self.pose[1])
                      >= self.min_target_dist] or free_candidates

        if not clusters:
            self.empty_checks += 1
            if self.empty_checks >= self.completion_checks:
                self.get_logger().info(
                    f'连续 {self.completion_checks} 次未发现 frontier, '
                    f'未知占比 {unknown_ratio * 100:.2f}% -> 探索完成')
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
            # 效用 = 真实信息增益 ÷ (1 + 行驶距离)
            # 增益估计: 站在 frontier 质心、以 2.5m 为半径能覆盖多少未知格
            # 这是 rrt_exploration / explore_lite 的经典做法,
            # 避免用"簇格子数"这种粗糙近似 —— 柱子附近的 frontier 虽然簇大,
            # 但激光被挡住后实际能揭示的未知格可能很少。
            # 黑名单区域乘 0.1: 强惩罚, 防止在同一区域反复探索。
            SENSOR_RADIUS = 2.5  # TurtleBot3 Lidar 射程 3.5m, 取保守值

            scored = []
            for c in candidates:
                dist = math.hypot(c[0] - self.pose[0], c[1] - self.pose[1])
                gain = self._information_gain_at(c[0], c[1], SENSOR_RADIUS)
                util = gain / (1.0 + dist)
                if self._in_blacklist_zone(c[0], c[1]):
                    util *= 0.1  # 强惩罚, 不再选刚离开的区域
                scored.append((util, gain, dist, c))

            scored.sort(key=lambda x: x[0], reverse=True)
            chosen = scored[0][3]
            chosen_gain = scored[0][1]
            chosen_util = scored[0][0]

            new_target = (chosen[0], chosen[1])
            moved = self.target is None or math.hypot(
                new_target[0] - self.target[0], new_target[1] - self.target[1]) > 0.3
            self.target = new_target
            if moved:
                self.target_time = now
                self.last_dist = None
                self.last_progress_time = now
                self.get_logger().info(
                    f'新目标 ({self.target[0]:.2f}, {self.target[1]:.2f}), '
                    f'距离 {math.hypot(self.target[0] - self.pose[0], self.target[1] - self.pose[1]):.2f} m, '
                    f'候选 {len(candidates)}, 估算 gain={chosen_gain} 未知格, 效用={chosen_util:.1f}, '
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

        # 到达后原地扫一圈, 让 SLAM 把这块画清楚再拉黑
        if self.arrive_deadline is not None:
            cmd.angular.z = self.w * 0.6
            self.cmd_pub.publish(cmd)
            return

        if dist < self.tol:
            self.get_logger().info(
                f'到达目标 {self.target}, 原地扫描 {self.arrive_pause:.1f}s')
            self.arrive_deadline = self.get_clock().now().nanoseconds \
                + int(self.arrive_pause * 1e9)
            cmd.angular.z = self.w * 0.6
            self.cmd_pub.publish(cmd)
            return

        now = self.get_clock().now()
        if self.last_dist is None or self.last_dist - dist > 0.30:
            self.last_dist = dist
            self.last_progress_time = now
        elif self.last_progress_time is not None and \
                (now - self.last_progress_time).nanoseconds * 1e-9 > self.stuck_time:
            self.cmd_pub.publish(Twist())
            self.give_up_target(f'{self.stuck_time:.0f}s 没有靠近 0.30m')
            return

        # 读取激光扇区 —— front 用 60° 宽 (±30°), 比之前 58° 略宽
        # 防止狭窄柱面刚好落在扇区间隙里被漏掉
        front = math.inf
        left = math.inf
        right = math.inf
        target_path_clear = math.inf
        if self.scan is not None:
            front = _min_range(self.scan.ranges, -0.52, 0.52,
                               self.scan.angle_increment, self.scan.angle_min)
            left = _min_range(self.scan.ranges, 0.52, 1.57,
                              self.scan.angle_increment, self.scan.angle_min)
            right = _min_range(self.scan.ranges, -1.57, -0.52,
                               self.scan.angle_increment, self.scan.angle_min)
            # 硬约束: 检查目标方向 ±30° 扇区内有没有障碍挡路
            # 哪怕正前方没扫到(比如柱子斜前方), 只要目标方向有就立刻绕
            t_start = max(err - 0.52, -1.57)
            t_end = min(err + 0.52, 1.57)
            if t_start < t_end:
                target_path_clear = _min_range(
                    self.scan.ranges, t_start, t_end,
                    self.scan.angle_increment, self.scan.angle_min)

        # ============== 分支优先级 ==============
        # 1. 贴柱极近: 完全停住, 只原地转开
        #    给柱子留足安全距离, 边转边走容易顶上去
        if front < 0.50 or (target_path_clear < 0.50 and abs(err) < 1.0):
            if self.turn_dir is None:
                self.turn_dir = 1 if left >= right else -1
                self.turn_since = now
            cmd.linear.x = 0.0
            cmd.angular.z = self.w * self.turn_dir * 1.0
            self.target_time = now
            self.last_progress_time = now

            # 解除: 前方≥0.75m 且目标方向也无威胁
            front_safe = front >= 0.75 and target_path_clear >= 0.75
            side_safe = left >= 0.75 or right >= 0.75
            if front_safe and side_safe:
                self.turn_dir = None
                self.turn_since = None

            if self.turn_since is not None and \
                    (now - self.turn_since).nanoseconds * 1e-9 > 15.0:
                self.turn_dir = None
                self.turn_since = None
                self.cmd_pub.publish(Twist())
                self.give_up_target('贴柱绕障 15s 未脱困')
                return

        # 2. 有一定距离但仍需绕障: 沿切线低速前进, 保持转向
        elif front < 0.75 or (self.turn_dir is not None and front < 1.0):
            if self.turn_dir is None:
                self.turn_dir = 1 if left >= right else -1
                self.turn_since = now
            cmd.linear.x = self.v * 0.35   # 0.07 m/s, 足够慢
            cmd.angular.z = self.w * self.turn_dir
            self.target_time = now
            self.last_progress_time = now

            front_clear = front >= 1.0 and target_path_clear >= 0.9
            side_clear = left >= 0.85 or right >= 0.85
            if front_clear and side_clear:
                self.turn_dir = None
                self.turn_since = None

            if self.turn_since is not None and \
                    (now - self.turn_since).nanoseconds * 1e-9 > 15.0:
                self.turn_dir = None
                self.turn_since = None
                self.cmd_pub.publish(Twist())
                self.give_up_target('绕障 15s 未脱困')
                return

        # 3. 目标方向上有障碍挡路: 直接进绕障, 不等正前方扫到
        elif target_path_clear < 1.0 and abs(err) < 1.0:
            if self.turn_dir is None:
                self.turn_dir = 1 if left >= right else -1
                self.turn_since = now
            cmd.linear.x = 0.0
            cmd.angular.z = self.w * self.turn_dir * 1.0
            self.target_time = now
            self.last_progress_time = now

            if target_path_clear >= 1.2:
                self.turn_dir = None
                self.turn_since = None

        # 4. 方向差大: 原地转到目标方向
        elif abs(err) > 0.6:
            self.turn_dir = None
            self.turn_since = None
            cmd.angular.z = self.w if err > 0 else -self.w

        # 5. 安全: 朝目标前进
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
