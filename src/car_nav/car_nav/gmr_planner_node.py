#!/usr/bin/env python3
"""GMR-RRT* 全局规划节点 —— 把论文算法接入 ROS 2 导航（核心演示）。

订阅:
  /map             (nav_msgs/OccupancyGrid)                    SLAM 静态地图
  /amcl_pose       (geometry_msgs/PoseWithCovarianceStamped)   当前位姿(起点)
  /gmr/goal_pose   (geometry_msgs/PoseStamped)                 本节点专用目标

  注意: 不要用 /goal_pose —— Humble 的 bt_navigator 也订阅该话题,
  会同时触发标准 NavFn 导航, 与本节点的 FollowPath 抢 /cmd_vel。
  RViz 里请用配置好的 "GMR Goal"(SetGoal -> /gmr/goal_pose) 工具;
  标准 "Nav2 Goal" 工具保留用于 NavFn 对比。

规划流程(论文完整链路, 代码零修改来自 02_复现):
  OccupancyGrid -> GridEnvironment
    -> A* 合成 demo 轨迹 (demos.py 流程)
    -> GMM 拟合 (K=8) -> GMR 回归 (gmr.py 式(1)-(6))
    -> GMR-RRT* 规划 (rrt_star.py, 论文 Algorithm 1+2)

发布 (均 TRANSIENT_LOCAL, RViz 后开也能看到):
  /gmr/path        (nav_msgs/Path)          最终路径(红色粗线)
  /gmr/tree        (MarkerArray/LINE_LIST)  RRT* 搜索树
  /gmr/demos       (MarkerArray/LINE_LIST)  demo 轨迹 + GMR 均值 + 协方差管道
  /gmr/navfn_path  (nav_msgs/Path)          对比用 NavFn 路径(compare_navfn:=true)

执行:
  FollowPath action (bt_navigator) —— 把自己的路径直接交给 Nav2,
  机器人沿 GMR-RRT* 路径行驶, DWB 只做局部跟踪避障。

用法:
  ros2 run car_nav gmr_planner_node
  ros2 run car_nav gmr_planner_node --ros-args -p compare_navfn:=true
  (在 RViz 用 "2D Goal Pose" 工具点目标即触发)
"""

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from car_nav.gmr_lib.gmr import fit_gmm, gmr_predict, GMRSampler
from car_nav.gmr_lib.rrt_star import RRTStar
from car_nav.grid_env import GridEnvironment, extract_path, generate_grid_demos

LATCHED = QoSProfile(depth=1,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class GmrPlanner(Node):

    def __init__(self):
        super().__init__('gmr_planner')

        # ---- 论文参数(按 612 单位 ≈ 12 m 等比缩放为米制) ----
        self.declare_parameter('n_demos', 10)          # 论文: 10 条演示
        self.declare_parameter('k', 8)                 # 论文: GMM K=8
        self.declare_parameter('step_size', 0.30)      # 论文: 6 / 612*12
        self.declare_parameter('rewire_radius', 1.2)   # 论文: 35 -> 1.2 m
        self.declare_parameter('max_iter', 1500)
        self.declare_parameter('goal_radius', 0.25)    # 论文: 14 -> 0.25 m
        self.declare_parameter('p_uniform', 0.5)       # 论文 Algorithm 2
        self.declare_parameter('demo_lateral', 0.4)    # 论文: 18 -> 0.4 m
        self.declare_parameter('demo_noise', 0.03)
        self.declare_parameter('seed', 0)
        # ---- 地图处理 ----
        self.declare_parameter('inflation_radius', 0.18)
        self.declare_parameter('occ_threshold', 50)
        self.declare_parameter('coarse_cell', 0.10)
        # ---- 行为 ----
        self.declare_parameter('follow_path', True)    # 规划后交给 Nav2 执行
        self.declare_parameter('compare_navfn', False) # 同时画 NavFn 对比

        # ---- 地图缓存 ----
        self.env = None            # GridEnvironment(不含起终点)
        self.origin = (0.0, 0.0)
        self.start_world = None    # (x, y) map frame
        self.map_msg = None

        self.create_subscription(
            OccupancyGrid, '/map', self.on_map, LATCHED)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.declare_parameter('goal_topic', '/gmr/goal_pose')
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value,
            self.on_goal, 10)

        self.path_pub = self.create_publisher(Path, '/gmr/path', LATCHED)
        self.tree_pub = self.create_publisher(MarkerArray, '/gmr/tree', LATCHED)
        self.demos_pub = self.create_publisher(MarkerArray, '/gmr/demos', LATCHED)
        self.navfn_pub = self.create_publisher(Path, '/gmr/navfn_path', LATCHED)

        self.follow_client = ActionClient(self, FollowPath, 'follow_path')
        self.navfn_client = ActionClient(self, ComputePathToPose,
                                         'compute_path_to_pose')

        # TF 兜底: /amcl_pose 空闲时不发布, 起点优先用 TF map->base_footprint
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 规划要跑十几秒; 放在工作线程里跑, 主线程照常 spin,
        # 这样 /amcl_pose、TF、FollowPath 反馈不会在规划期间停更
        self.planning = False
        self.plan_thread = None

        self.get_logger().info(
            'gmr_planner 已启动, 等待 /map; 在 RViz 用 "GMR Goal" 工具'
            '(发布 ' + self.get_parameter('goal_topic').value + ') 下发目标')

    # ------------------------------------------------------------------ 回调
    def on_map(self, msg):
        if self.map_msg is not None:
            return                      # 地图不变, 只建一次环境
        self.map_msg = msg
        info = msg.info
        self.origin = (info.origin.position.x, info.origin.position.y)
        p = self.get_parameter
        self.env = GridEnvironment(
            'slam_map', msg.data, info.width, info.height,
            info.resolution,
            inflation_radius=float(p('inflation_radius').value),
            occ_threshold=int(p('occ_threshold').value))
        self.get_logger().info(
            f'地图已加载: {info.width}x{info.height} res={info.resolution:.3f} '
            f'origin=({self.origin[0]:.1f},{self.origin[1]:.1f}), '
            f'可通行栅格占比 {self.env._free.mean():.0%}')

    def on_pose(self, msg):
        q = msg.pose.pose
        self.start_world = (q.position.x, q.position.y)

    def on_goal(self, msg):
        if self.env is None:
            self.get_logger().warn('还没有收到 /map, 忽略目标')
            return
        if self.planning:
            self.get_logger().warn(
                '上一次规划还在进行(约十几秒), 本次目标先忽略, 请等路径出来再点')
            return
        self.start_world = self.get_start_pose()
        if self.start_world is None:
            self.get_logger().warn('起点不可用(无 /amcl_pose 且 TF 查询失败), 忽略目标')
            return
        self.planning = True
        self.plan_thread = threading.Thread(
            target=self._plan_worker, args=(msg,), daemon=True)
        self.plan_thread.start()

    def _plan_worker(self, goal_msg):
        try:
            self.plan(goal_msg)
        except Exception as e:                      # noqa: BLE001 演示节点
            self.get_logger().error(f'规划失败: {e}')
        finally:
            self.planning = False

    def get_start_pose(self):
        """起点: 优先 /amcl_pose 缓存, 否则 TF map->base_footprint。"""
        if self.start_world is not None:
            return self.start_world
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    # ------------------------------------------------------------------ 规划
    def plan(self, goal_msg):
        import time
        p = self.get_parameter
        env = self.env
        ox, oy = self.origin

        # 1. 起终点: world -> env 坐标, 并吸附到最近自由栅格
        start_env = np.array([self.start_world[0] - ox,
                              self.start_world[1] - oy])
        goal_env = np.array([goal_msg.pose.position.x - ox,
                             goal_msg.pose.position.y - oy])
        env.start = env.nearest_free(start_env)
        env.goal = env.nearest_free(goal_env)
        if env.start is None or env.goal is None:
            self.get_logger().error('起点或目标不在自由空间内')
            return
        env.goal_radius = float(p('goal_radius').value)
        dist = float(np.linalg.norm(env.goal - env.start))
        self.get_logger().info(
            f'=== 收到目标: 起点世界({self.start_world[0]:.2f},'
            f'{self.start_world[1]:.2f}) -> 终点世界('
            f'{goal_msg.pose.position.x:.2f},{goal_msg.pose.position.y:.2f}), '
            f'直线距离 {dist:.2f} m ===')

        # 2. demo 轨迹(A* 参考路径 + 横向偏移, 对应论文的 10 条人工演示)
        t0 = time.perf_counter()
        demos, ref_len = generate_grid_demos(
            env, n_demos=int(p('n_demos').value), seed=int(p('seed').value),
            lateral=float(p('demo_lateral').value),
            noise=float(p('demo_noise').value),
            coarse_cell=float(p('coarse_cell').value))
        t_demos = time.perf_counter() - t0
        self.get_logger().info(
            f'[1/3] demo 轨迹: {len(demos)} 条 x {demos[0].shape[0]} 点 '
            f'(A* 参考 {ref_len:.1f} m), 耗时 {t_demos:.2f} s')

        # 3. GMM 拟合 + GMR 回归(论文式(1)-(6))
        gmm, n = fit_gmm(demos, n_components=int(p('k').value),
                         seed=int(p('seed').value))
        mu, sigma = gmr_predict(gmm, n)
        self.get_logger().info(
            f'[2/3] GMM(K={gmm.n_components}) 拟合 + GMR 回归完成, '
            f'耗时 {time.perf_counter() - t0 - t_demos:.2f} s')

        # 4. GMR-RRT*(论文 Algorithm 1 主干 + Algorithm 2 采样)
        rng = np.random.default_rng(int(p('seed').value))
        sampler = GMRSampler(mu, sigma, env, rng,
                             p_uniform=float(p('p_uniform').value))
        planner = RRTStar(env, sampler,
                          step_size=float(p('step_size').value),
                          rewire_radius=float(p('rewire_radius').value),
                          max_iter=int(p('max_iter').value))
        res = planner.plan()
        path_env = extract_path(res, env)
        if path_env is None:
            self.get_logger().error(
                f'GMR-RRT* 未找到路径 (迭代 {res["n_nodes"]} 节点), '
                '可尝试增大 max_iter 或检查目标可达性')
            return
        self.get_logger().info(
            f'[3/3] GMR-RRT*: 首次成功迭代 {res["iter_init"]}, '
            f'节点 {res["n_nodes"]}, 路径长 {res["cost_final"]:.1f} m, '
            f'总耗时 {time.perf_counter() - t0:.2f} s')

        # 5. 发布可视化(env -> world 坐标)
        self.publish_path(path_env)
        self.publish_tree(res)
        self.publish_demos(demos, mu, sigma)

        # 6. 交给 Nav2 执行(机器人沿 GMR-RRT* 路径行驶)
        if bool(p('follow_path').value):
            self.send_follow_path(path_env, goal_msg)
        if bool(p('compare_navfn').value):
            self.request_navfn(goal_msg)

    # ------------------------------------------------------------------ 发布
    def _now(self):
        return self.get_clock().now().to_msg()

    def publish_path(self, path_env):
        ox, oy = self.origin
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self._now()
        for x, y in path_env:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x + ox)
            ps.pose.position.y = float(y + oy)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def publish_tree(self, res):
        ox, oy = self.origin
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self._now()
        m.ns = 'gmr_tree'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.015
        m.color.r, m.color.g, m.color.b, m.color.a = 0.35, 0.6, 1.0, 0.45
        pts, par = res['pts'], res['par']
        for i in range(1, len(pts)):
            j = int(par[i])
            if j < 0:
                continue
            for (x, y) in (pts[j], pts[i]):
                pt = Point()
                pt.x, pt.y = float(x + ox), float(y + oy)
                m.points.append(pt)
        arr = MarkerArray()
        arr.markers.append(m)
        self.tree_pub.publish(arr)

    def publish_demos(self, demos, mu, sigma):
        ox, oy = self.origin
        arr = MarkerArray()

        # demo 轨迹(蓝)
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self._now()
        m.ns = 'gmr_demos'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = 0.25, 0.45, 0.95, 0.6
        for d in demos:
            for a, b in zip(d[:-1], d[1:]):
                for (x, y) in (a, b):
                    pt = Point()
                    pt.x, pt.y = float(x + ox), float(y + oy)
                    m.points.append(pt)
        arr.markers.append(m)

        # GMR 均值轨迹(品红)
        m2 = Marker()
        m2.header = m.header
        m2.ns = 'gmr_mean'
        m2.id = 1
        m2.type = Marker.LINE_STRIP
        m2.action = Marker.ADD
        m2.scale.x = 0.035
        m2.color.r, m2.color.g, m2.color.b, m2.color.a = 1.0, 0.15, 0.85, 0.95
        for x, y in mu:
            pt = Point()
            pt.x, pt.y = float(x + ox), float(y + oy)
            m2.points.append(pt)
        arr.markers.append(m2)

        # 协方差管道(椭圆环, 对标论文 Fig.4(d))
        m3 = Marker()
        m3.header = m.header
        m3.ns = 'gmr_cov'
        m3.id = 2
        m3.type = Marker.LINE_LIST
        m3.action = Marker.ADD
        m3.scale.x = 0.01
        m3.color.r, m3.color.g, m3.color.b, m3.color.a = 1.0, 0.15, 0.85, 0.4
        step = max(1, len(mu) // 22)
        for i in range(0, len(mu), step):
            vals, vecs = np.linalg.eigh(0.5 * (sigma[i] + sigma[i].T))
            vals = np.maximum(vals, 1e-6)
            for k in range(32):
                a1 = 2 * math.pi * k / 32
                a2 = 2 * math.pi * (k + 1) / 32
                for ang in (a1, a2):
                    d = vecs @ (np.sqrt(vals) * np.array(
                        [math.cos(ang), math.sin(ang)]))
                    pt = Point()
                    pt.x = float(mu[i, 0] + d[0] + ox)
                    pt.y = float(mu[i, 1] + d[1] + oy)
                    m3.points.append(pt)
        arr.markers.append(m3)
        self.demos_pub.publish(arr)

    # ------------------------------------------------------------------ 执行
    def _wait_server(self, client, timeout=2.0):
        """在工作线程里等 action server。

        不能直接用 client.wait_for_server(): 它会自己 spin 节点, 与主线程的
        executor 冲突(Node has already been added to an executor)。
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if client.server_is_ready():
                return True
            time.sleep(0.05)
        return client.server_is_ready()

    def send_follow_path(self, path_env, goal_msg):
        if not self._wait_server(self.follow_client):
            self.get_logger().warn('follow_path action 不可用(Nav2 未激活?), '
                                   '只发布路径不执行')
            return
        ox, oy = self.origin
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self._now()
        for i, (x, y) in enumerate(path_env):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x + ox)
            ps.pose.position.y = float(y + oy)
            if i == len(path_env) - 1:
                ps.pose.orientation = goal_msg.pose.orientation
            else:
                ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        goal = FollowPath.Goal()
        goal.path = path
        future = self.follow_client.send_goal_async(
            goal, feedback_callback=self.on_follow_feedback)
        future.add_done_callback(self.on_follow_goal_response)
        self.get_logger().info('>>> FollowPath 已下发, 机器人开始沿 GMR-RRT* '
                               '路径行驶 (DWB 局部跟踪)')

    def on_follow_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('FollowPath 目标被拒绝')
            return
        handle.get_result_async().add_done_callback(self.on_follow_result)

    def on_follow_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        d = getattr(fb, 'distance_to_goal', None)
        if d is None:
            d = getattr(fb, 'distance_remaining', -1.0)
        self.get_logger().info(f'    跟踪中, 剩余 {d:.2f} m',
                               throttle_duration_sec=3.0)

    def on_follow_result(self, future):
        res = future.result()
        status = res.status if res is not None else -1
        if status == 4:      # SUCCEEDED
            self.get_logger().info('✓ 到达目标, GMR-RRT* 路径执行完成')
        else:
            self.get_logger().warn(f'FollowPath 结束, status={status}')

    def request_navfn(self, goal_msg):
        """对比模式: 请求 Nav2 自带 NavFn 规划同一起终点, 画在 /gmr/navfn_path。"""
        if not self._wait_server(self.navfn_client):
            self.get_logger().warn('compute_path_to_pose 不可用, 跳过对比')
            return
        goal = ComputePathToPose.Goal()
        goal.goal = goal_msg
        goal.planner_id = 'GridBased'
        future = self.navfn_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_navfn_accepted(f, goal_msg))

    def _on_navfn_accepted(self, future, goal_msg):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('NavFn 对比请求被拒绝')
            return
        handle.get_result_async().add_done_callback(self._on_navfn_result)

    def _on_navfn_result(self, future):
        res = future.result().result
        if res is None or len(res.path.poses) == 0:
            self.get_logger().warn('NavFn 未返回路径')
            return
        self.navfn_pub.publish(res.path)
        self.get_logger().info('NavFn 对比路径已发布到 /gmr/navfn_path')


def main(args=None):
    rclpy.init(args=args)
    node = GmrPlanner()
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
