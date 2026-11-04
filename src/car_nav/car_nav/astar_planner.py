#!/usr/bin/env python3
"""自写 A* 全局规划节点。

订阅:
  /map         (nav_msgs/OccupancyGrid)                    静态地图 (latched)
  /amcl_pose   (geometry_msgs/PoseWithCovarianceStamped)   当前位姿
  /goal_pose   (geometry_msgs/PoseStamped)                 RViz "2D Goal Pose"

发布:
  /car_nav/plan     (nav_msgs/Path)                 自写 A* 规划出的路径
  /car_nav/explored (visualization_msgs/Marker)     搜索过的栅格, 便于观察
"""

import heapq
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

SQRT2 = math.sqrt(2.0)
INF = float('inf')


class AstarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('plan_topic', '/car_nav/plan')
        self.declare_parameter('explored_topic', '/car_nav/explored')
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('inflation_radius', 0.25)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('replan_period', 1.0)

        p = self.get_parameter
        self.inflation_radius = p('inflation_radius').value
        self.occ_threshold = p('occupied_threshold').value
        self.allow_diagonal = p('allow_diagonal').value

        # 地图与栅格数据
        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.free = None          # bool 数组, True = 可通行(已膨胀)
        self.explored_cells = []

        self.start_pose = None    # (x, y)
        self.goal_pose = None     # (x, y)
        self.current_path = None  # list[(x, y)]

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            OccupancyGrid, p('map_topic').value, self.on_map, map_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, p('pose_topic').value, self.on_pose, 10)
        self.create_subscription(
            PoseStamped, p('goal_topic').value, self.on_goal, 10)

        self.plan_pub = self.create_publisher(Path, p('plan_topic').value, 1)
        self.marker_pub = self.create_publisher(
            Marker, p('explored_topic').value, 1)

        self.create_timer(p('replan_period').value, self.publish_cached)
        self.get_logger().info('astar_planner 已启动, 等待 /map 与目标点')

    # ------------------------------------------------------------------ 回调
    def on_map(self, msg):
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape(
            (self.height, self.width))
        occupied = data >= self.occ_threshold
        self.free = ~self._inflate(occupied, self.inflation_radius)
        self.get_logger().info(
            f'地图已加载: {self.width}x{self.height} '
            f'res={self.resolution:.3f} 可通行栅格={int(self.free.sum())}')

    def on_pose(self, msg):
        q = msg.pose.pose
        self.start_pose = (q.position.x, q.position.y)

    def on_goal(self, msg):
        q = msg.pose
        self.goal_pose = (q.position.x, q.position.y)
        self.get_logger().info(
            f'收到目标 ({self.goal_pose[0]:.2f}, {self.goal_pose[1]:.2f}), 开始 A*')
        self.plan()

    # ------------------------------------------------------------------ 规划
    def plan(self):
        if self.free is None or self.start_pose is None or self.goal_pose is None:
            self.get_logger().warn('缺少地图或起点/终点, 暂不规划')
            return

        start = self._world_to_grid(*self.start_pose)
        goal = self._world_to_grid(*self.goal_pose)
        if start is None or goal is None:
            self.get_logger().warn('起点/终点超出地图范围')
            return

        start = self._nearest_free(start)
        goal = self._nearest_free(goal)
        if start is None or goal is None:
            self.get_logger().warn('起点/终点不可通行且附近无空闲栅格')
            return

        cells = self._astar(start, goal)
        if cells is None:
            self.get_logger().warn('A* 未找到可行路径')
            return

        self.current_path = [self._grid_to_world(c) for c in cells]
        self.get_logger().info(
            f'A* 规划成功: {len(self.current_path)} 个路径点, '
            f'长度 {self._path_length(self.current_path):.2f} m')
        self.publish_cached()
        self.publish_explored()

    def _astar(self, start, goal):
        w, h = self.width, self.height
        free = self.free

        offsets = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]
        if self.allow_diagonal:
            offsets += [(-1, -1, SQRT2), (-1, 1, SQRT2),
                        (1, -1, SQRT2), (1, 1, SQRT2)]

        sx, sy = start
        gx, gy = goal
        start_i = sy * w + sx
        goal_i = gy * w + gx

        g_score = {start_i: 0.0}
        came_from = {}
        closed = set()
        heap = [(self._heuristic(sx, sy, gx, gy), start_i)]
        visited = []

        while heap:
            _, cur = heapq.heappop(heap)
            if cur == goal_i:
                closed.add(cur)
                visited.append(cur)
                break
            if cur in closed:
                continue
            closed.add(cur)
            visited.append(cur)

            cx, cy = cur % w, cur // w
            for dx, dy, cost in offsets:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                if not free[ny, nx]:
                    continue
                # 禁止斜穿墙角
                if dx != 0 and dy != 0 and not (free[cy, nx] and free[ny, cx]):
                    continue
                ni = ny * w + nx
                if ni in closed:
                    continue
                ng = g_score[cur] + cost
                if ng < g_score.get(ni, INF):
                    g_score[ni] = ng
                    came_from[ni] = cur
                    heapq.heappush(
                        heap, (ng + self._heuristic(nx, ny, gx, gy), ni))

        self.explored_cells = visited
        if goal_i not in came_from and goal_i != start_i:
            return None

        path = [goal_i]
        while path[-1] != start_i:
            path.append(came_from[path[-1]])
        path.reverse()
        return [(i % w, i // w) for i in path]

    def _heuristic(self, x, y, gx, gy):
        dx, dy = abs(x - gx), abs(y - gy)
        if self.allow_diagonal:
            return max(dx, dy) + (SQRT2 - 1.0) * min(dx, dy)
        return dx + dy

    # ------------------------------------------------------------------ 发布
    def publish_cached(self):
        if not self.current_path:
            return
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.current_path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.plan_pub.publish(msg)

    def publish_explored(self):
        if not self.explored_cells:
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'astar'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.35

        for i in self.explored_cells[::4]:
            x, y = self._grid_to_world((i % self.width, i // self.width))
            pt = Point()
            pt.x, pt.y, pt.z = x, y, 0.02
            marker.points.append(pt)
        self.marker_pub.publish(marker)

    # ------------------------------------------------------------------ 工具
    def _inflate(self, occupied, radius):
        r = int(math.ceil(radius / self.resolution)) if self.resolution else 0
        if r <= 0:
            return occupied
        h, w = occupied.shape
        inflated = np.zeros_like(occupied)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                ys_dst = slice(max(0, dy), h + min(0, dy))
                xs_dst = slice(max(0, dx), w + min(0, dx))
                ys_src = slice(max(0, -dy), h + min(0, -dy))
                xs_src = slice(max(0, -dx), w + min(0, -dx))
                inflated[ys_dst, xs_dst] |= occupied[ys_src, xs_src]
        return inflated

    def _world_to_grid(self, x, y):
        mx = int(math.floor((x - self.origin_x) / self.resolution))
        my = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= mx < self.width and 0 <= my < self.height:
            return (mx, my)
        return None

    def _grid_to_world(self, cell):
        mx, my = cell
        return (self.origin_x + (mx + 0.5) * self.resolution,
                self.origin_y + (my + 0.5) * self.resolution)

    def _nearest_free(self, cell, max_radius=40):
        mx, my = cell
        if self.free[my, mx]:
            return cell
        for r in range(1, max_radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    nx, ny = mx + dx, my + dy
                    if (0 <= nx < self.width and 0 <= ny < self.height
                            and self.free[ny, nx]):
                        return (nx, ny)
        return None

    @staticmethod
    def _path_length(path):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(path[:-1], path[1:]))


def main(args=None):
    rclpy.init(args=args)
    node = AstarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
