#!/usr/bin/env python3
"""OccupancyGrid -> 论文 Environment 适配层（零修改接入 gmr_lib）。

把 ROS 的 nav_msgs/OccupancyGrid 包装成 gmr_lib.map_env.Environment 的子类,
让 rrt_star.py / gmr.py / demos.py 的论文代码原样运行在真实 SLAM 地图上。

坐标约定:
  - 内部坐标 = 世界坐标(map frame) 平移 origin 后的坐标, 范围 [0, size],
    与论文代码的 [0, size] 约定一致, 因此采样器无需任何修改。
  - 节点在入口处做 world <-> env 坐标转换。
  - 单位: 米。论文参数按比例缩放(612 单位 ≈ 12 m):
      step 6 -> 0.30 m, rewire 35 -> 1.2 m, goal radius 14 -> 0.25 m
"""

import collections
import math

import numpy as np

from car_nav.gmr_lib.demos import (_astar, _is_valid_demo, _lateral_offsets,
                                   _smooth, resample_polyline)
from car_nav.gmr_lib.map_env import Environment


def extract_path(res, env):
    """从 RRT* 树里回溯到目标的最优路径（与 02_复现/run_experiment.py 一致）。"""
    if res["goal_parent"] < 0:
        return None
    pts, par = res["pts"], res["par"]
    idx, path = int(res["goal_parent"]), [np.asarray(env.goal, dtype=float)]
    while idx >= 0:
        path.append(pts[idx])
        idx = int(par[idx])
    return np.array(path[::-1])


def _inflate_grid(occupied, radius_cells):
    """圆形结构元膨胀(向量化)。occupied [H,W] bool -> inflated [H,W] bool。"""
    r = int(np.ceil(radius_cells))
    if r <= 0:
        return occupied.copy()
    h, w = occupied.shape
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    disk = (yy * yy + xx * xx) <= radius_cells * radius_cells
    inflated = np.zeros_like(occupied)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if not disk[dy + r, dx + r]:
                continue
            ys_dst = slice(max(0, dy), h + min(0, dy))
            xs_dst = slice(max(0, dx), w + min(0, dx))
            ys_src = slice(max(0, -dy), h + min(0, -dy))
            xs_src = slice(max(0, -dx), w + min(0, -dx))
            inflated[ys_dst, xs_dst] |= occupied[ys_src, xs_src]
    return inflated


class GridEnvironment(Environment):
    """占据栅格版环境。

    is_free_point / is_free_segment 用栅格查询 + 线段采样代替
    论文代码的矩形障碍 + Liang-Barsky, 接口完全兼容。
    """

    def __init__(self, name, grid_data, width, height, resolution,
                 inflation_radius=0.18, occ_threshold=50,
                 unknown_as_occupied=False):
        super().__init__(name, size=1.0)
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.occ_threshold = int(occ_threshold)

        data = np.asarray(grid_data, dtype=np.int16)
        if data.ndim == 1:
            data = data.reshape(self.height, self.width)
        # -1 = 未知; >= threshold = 占据。
        # 未知按自由处理(与 Nav2 allow_unknown 默认一致): 已映射障碍靠膨胀规避,
        # 未映射区域交给 DWB 局部避障兜底。
        occupied = data >= self.occ_threshold
        if unknown_as_occupied:
            occupied |= data < 0
        radius_cells = inflation_radius / self.resolution
        self._free = ~_inflate_grid(occupied, radius_cells)   # [row=y, col=x]

        # 供 UniformSampler 使用: 覆盖整幅地图(越界采样会被 is_free_point 拒绝)
        self.size = max(self.width, self.height) * self.resolution
        self.n_demo_points = 100        # 由 generate_grid_demos 按路径长度覆盖

    # ---------------------------------------------------------- 查询接口
    def _to_cell(self, x, y):
        col = int(math.floor(x / self.resolution))
        row = int(math.floor(y / self.resolution))
        if 0 <= col < self.width and 0 <= row < self.height:
            return row, col
        return None

    def is_free_point(self, p):
        cell = self._to_cell(float(p[0]), float(p[1]))
        if cell is None:
            return False
        return bool(self._free[cell])

    def is_free_segment(self, p1, p2):
        """线段无碰撞: 沿线段以 resolution/2 步长采样检查栅格。"""
        x1, y1 = float(p1[0]), float(p1[1])
        x2, y2 = float(p2[0]), float(p2[1])
        d = math.hypot(x2 - x1, y2 - y1)
        if d <= 1e-9:
            return self.is_free_point(p2)
        n = max(2, int(d / (self.resolution * 0.5)) + 1)
        for i in range(n + 1):
            t = i / n
            if not self.is_free_point((x1 + (x2 - x1) * t, y1 + (y2 - y1) * t)):
                return False
        return True

    # ---------------------------------------------------------- 工具
    def nearest_free(self, p, max_radius_m=1.5):
        """把点挪到最近的自由栅格中心(米制 env 坐标), 找不到返回 None。"""
        cell = self._to_cell(float(p[0]), float(p[1]))
        if cell is None:
            return None
        r0, c0 = cell
        r_max = int(max_radius_m / self.resolution)
        for r in range(0, r_max + 1):
            for row in range(r0 - r, r0 + r + 1):
                for col in range(c0 - r, c0 + r + 1):
                    if max(abs(row - r0), abs(col - c0)) != r:
                        continue
                    if 0 <= row < self.height and 0 <= col < self.width \
                            and self._free[row, col]:
                        return np.array([(col + 0.5) * self.resolution,
                                         (row + 0.5) * self.resolution])
        return None

    def coarse_grid(self, cell_size):
        """降采样自由栅格(粗分辨率, 供 A* 用)。

        cell_size 为米。返回 (free, (nh, nw))。
        粗格自由当且仅当其覆盖的所有细格都自由(保守)。
        """
        f = max(1, int(round(cell_size / self.resolution)))
        h = (self.height // f) * f
        w = (self.width // f) * f
        blk = self._free[:h, :w].reshape(h // f, f, w // f, f)
        return blk.all(axis=(1, 3)), (h // f, w // f)


# ------------------------------------------------------------------ 演示生成

def _nearest_free_cell(cell, free):
    """BFS 找最近自由格(粗栅格), 返回 (row, col) 或 None。"""
    if cell is None:
        return None
    nh, nw = free.shape
    q = collections.deque([cell])
    seen = {cell}
    while q:
        r, c = q.popleft()
        if free[r, c]:
            return (r, c)
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < nh and 0 <= nc < nw and (nr, nc) not in seen:
                seen.add((nr, nc))
                q.append((nr, nc))
        if len(seen) > 20000:      # 安全阀
            break
    return None


def generate_grid_demos(env, n_demos=10, n_points=None, seed=0,
                        lateral=0.4, noise=0.03, n_ctrl=5, coarse_cell=0.1):
    """在栅格环境上合成演示轨迹。

    与 gmr_lib.demos.generate_demos 流程一致(A* 参考路径 -> 平滑 -> 弧长重采样
    -> 横向偏移 + 噪声 -> 碰撞校验), 区别是直接用环境自带的占据栅格做 A*,
    而非从矩形障碍重新栅格化。

    返回 (demos, ref_length)。
    """
    rng = np.random.default_rng(seed)
    free, (nh, nw) = env.coarse_grid(coarse_cell)

    def to_cell(p):
        col = int(math.floor(p[0] / coarse_cell))
        row = int(math.floor(p[1] / coarse_cell))
        if 0 <= col < nw and 0 <= row < nh:
            return (row, col)
        return None

    s = _nearest_free_cell(to_cell(env.start), free)
    g = _nearest_free_cell(to_cell(env.goal), free)
    if s is None or g is None:
        raise RuntimeError('起点/目标附近找不到自由栅格')

    # demos._astar 假设方形栅格: 非方形时用占据格补齐到正方形
    n_sq = max(nh, nw)
    free_sq = np.zeros((n_sq, n_sq), dtype=bool)
    free_sq[:nh, :nw] = free
    path = _astar(free_sq, n_sq, s, g, rng, perturb=0.0)
    if path is None:
        raise RuntimeError('A* 找不到参考路径, 检查地图连通性')

    pts = np.array([[(j + 0.5) * coarse_cell, (i + 0.5) * coarse_cell]
                    for i, j in path], dtype=float)
    pts = _smooth(pts, env)
    ref_len = float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())

    if n_points is None:
        n_points = int(np.clip(ref_len / 0.15, 40, 160))
    ref = resample_polyline(pts, n_points)
    ref[0], ref[-1] = np.asarray(env.start, float), np.asarray(env.goal, float)
    env.n_demo_points = n_points

    demos = []
    for k in range(n_demos):
        amp = 0.0 if k == 0 else lateral
        demo = ref.copy()
        for _ in range(8):
            cand = ref + _lateral_offsets(ref, amp, n_ctrl, rng)
            cand = cand + rng.normal(0.0, noise, size=cand.shape)
            cand[0], cand[-1] = np.asarray(env.start, float), \
                np.asarray(env.goal, float)
            if _is_valid_demo(cand, env):
                demo = cand
                break
            amp *= 0.5
        demos.append(demo)
    return demos, ref_len
