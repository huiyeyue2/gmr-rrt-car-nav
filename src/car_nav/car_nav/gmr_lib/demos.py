"""演示轨迹生成。

论文假设已经有 10 条人工演示（真机遥操作采集）。这里没有真机，
改用「网格 A* 参考路径 + 平滑 + 扰动」合成演示：效果等价——
给出「人在这张地图上会怎么走」的轨迹分布。

流程：栅格化地图 -> A* 求一条无碰撞参考路径 -> 平滑 -> 按弧长等距重采样到 N 点
      -> 叠加高斯噪声 -> 碰撞校验（不合格则减小噪声重试）
"""

import heapq
import numpy as np

CELL = 6.0          # 与论文 step size 一致
SMOOTH_WIN = 9

# 8 邻域 (di, dj, 步长权重)
_NEIGH = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
          (1, 1, 1.41421356), (1, -1, 1.41421356),
          (-1, 1, 1.41421356), (-1, -1, 1.41421356)]


def _free_grid(env, cell=CELL):
    """把地图栅格化，返回自由格布尔矩阵与格数。"""
    n = int(round(env.size / cell))
    xs = (np.arange(n) + 0.5) * cell
    xx, yy = np.meshgrid(xs, xs, indexing="ij")
    free = (xx > 0) & (xx < env.size) & (yy > 0) & (yy < env.size)
    for ob in env.obstacles:
        inside = ((xx >= ob.xmin) & (xx <= ob.xmax) &
                  (yy >= ob.ymin) & (yy <= ob.ymax))
        free &= ~inside
    return free, n


def _nearest_free_cell(p, free, cell):
    """找离点 p 最近的自由格。"""
    idx = np.argwhere(free)
    if idx.size == 0:
        raise RuntimeError("地图没有自由格")
    centers = (idx + 0.5) * cell
    d = ((centers[:, 0] - p[0]) ** 2 + (centers[:, 1] - p[1]) ** 2)
    return tuple(idx[int(np.argmin(d))])


def _astar(free, n, s, g, rng, perturb=0.0):
    """8 邻域 A*，禁止切角。perturb>0 时给格子加随机代价，用于生成多样路径。"""
    noise = 1.0 + perturb * rng.random((n, n)) if perturb > 0 else None
    gscore = np.full((n, n), np.inf)
    gscore[s] = 0.0
    came = {}
    h = lambda a: float(np.hypot(a[0] - g[0], a[1] - g[1]))
    open_ = [(h(s), 0.0, s)]
    closed = np.zeros((n, n), dtype=bool)
    while open_:
        _, gc, u = heapq.heappop(open_)
        if u == g:
            path = [u]
            while path[-1] in came:
                path.append(came[path[-1]])
            return path[::-1]
        if closed[u]:
            continue
        closed[u] = True
        for di, dj, w in _NEIGH:
            vi, vj = u[0] + di, u[1] + dj
            if not (0 <= vi < n and 0 <= vj < n) or not free[vi, vj]:
                continue
            if di != 0 and dj != 0:                      # 不许切角
                if not (free[u[0] + di, u[1]] and free[u[0], u[1] + dj]):
                    continue
            if closed[vi, vj]:
                continue
            ng = gc + w * (noise[vi, vj] if noise is not None else 1.0)
            if ng < gscore[vi, vj] - 1e-9:
                gscore[vi, vj] = ng
                came[(vi, vj)] = u
                heapq.heappush(open_, (ng + h((vi, vj)), ng, (vi, vj)))
    return None


def _smooth(points, env, window=SMOOTH_WIN, iters=4):
    """滑动平均平滑，每轮都做碰撞校验，不平滑就停。"""
    pts = np.asarray(points, dtype=float)
    if len(pts) <= window:
        return pts
    for _ in range(iters):
        ker = np.ones(window) / window
        sm = pts.copy()
        sm[:, 0] = np.convolve(pts[:, 0], ker, mode="same")
        sm[:, 1] = np.convolve(pts[:, 1], ker, mode="same")
        sm[0], sm[-1] = pts[0], pts[-1]
        if all(env.is_free_segment(sm[i], sm[i + 1]) for i in range(len(sm) - 1)):
            pts = sm
        else:
            break
    return pts


def resample_polyline(points, n):
    """按弧长等距重采样到 n 个点。"""
    pts = np.asarray(points, dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 0:
        return np.repeat(pts[:1], n, axis=0)
    t = np.linspace(0.0, s[-1], n)
    return np.stack([np.interp(t, s, pts[:, 0]),
                     np.interp(t, s, pts[:, 1])], axis=1)


def _is_valid_demo(demo, env):
    if not env.is_free_point(demo[0]) or not env.is_free_point(demo[-1]):
        return False
    return all(env.is_free_segment(demo[i], demo[i + 1])
               for i in range(len(demo) - 1))


def _lateral_offsets(ref, amp, n_ctrl, rng):
    """沿参考路径的法向生成低频偏移量（起终点处强制为 0）。"""
    n = len(ref)
    d = np.gradient(ref, axis=0)
    tn = d / (np.linalg.norm(d, axis=1, keepdims=True) + 1e-9)
    normal = np.stack([-tn[:, 1], tn[:, 0]], axis=1)
    if amp <= 0:
        return np.zeros((n, 2))
    ctrl_t = np.linspace(0, n - 1, n_ctrl)
    ctrl_v = rng.normal(0.0, amp, n_ctrl)
    ctrl_v[0] = ctrl_v[-1] = 0.0
    off = np.interp(np.arange(n), ctrl_t, ctrl_v)
    return normal * off[:, None]


def generate_demos(env, n_demos=10, n_points=None, seed=0,
                   lateral=18.0, noise=1.5, n_ctrl=5, cell=CELL):
    """生成 n_demos 条无碰撞演示，每条重采样到 n_points 个点。

    所有演示沿同一条参考走廊做低频横向偏移 —— 对应论文设定的
    「同一任务被重复执行多次」，而不是每条约走不同的路。
    起点 / 终点固定在 env.start / env.goal。
    """
    n_points = n_points or env.n_demo_points
    rng = np.random.default_rng(seed)
    free, n = _free_grid(env, cell)
    s_cell = _nearest_free_cell(env.start, free, cell)
    g_cell = _nearest_free_cell(env.goal, free, cell)

    path = _astar(free, n, s_cell, g_cell, rng, perturb=0.0)
    if path is None:
        raise RuntimeError(
            f"{env.name}: A* 找不到从起点到目标的路径，请检查障碍布局")
    pts = np.array([[(i + 0.5) * cell, (j + 0.5) * cell] for i, j in path])
    pts = _smooth(pts, env)
    ref = resample_polyline(pts, n_points)
    ref[0], ref[-1] = env.start, env.goal

    demos = []
    for k in range(n_demos):
        amp = 0.0 if k == 0 else lateral      # 第 1 条就是参考路径本身
        demo = ref.copy()
        for _ in range(8):
            cand = ref + _lateral_offsets(ref, amp, n_ctrl, rng)
            cand = cand + rng.normal(0.0, noise, size=cand.shape)
            cand[0], cand[-1] = env.start, env.goal
            if _is_valid_demo(cand, env):
                demo = cand
                break
            amp *= 0.5                        # 撞障碍就缩小偏移重试
        demos.append(demo)

    return demos


if __name__ == "__main__":
    import os
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from map_env import ALL_ENVS, draw_env

    fig, axes = plt.subplots(1, 3, figsize=(15, 5.2))
    for ax, factory in zip(axes, ALL_ENVS):
        env = factory()
        demos = generate_demos(env, n_demos=10, seed=0)
        draw_env(ax, env)
        for k, d in enumerate(demos):
            ax.plot(d[:, 0], d[:, 1], "-", color="royalblue",
                    lw=1.0, alpha=0.75, zorder=4)
        ax.set_title(f"{env.name}   10 demos x {env.n_demo_points} pts")
    plt.tight_layout()
    os.makedirs("结果", exist_ok=True)
    plt.savefig(os.path.join("结果", "check_demos.png"), dpi=110)
    print("已保存 结果/check_demos.png")
