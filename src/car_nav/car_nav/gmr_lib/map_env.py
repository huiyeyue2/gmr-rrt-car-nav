"""2D 环境与碰撞检测。

坐标约定：像素坐标，x 向右，y 向上，范围 [0, size]。
仿真设置对齐论文 GMR-RRT* (IEEE T-IV 2022)：
    map size  = 612
    step size = 6
    rewire r  = 35
    goal r    = 14
"""

import numpy as np


class Obstacle:
    """轴对齐矩形障碍。"""

    def __init__(self, x, y, w, h):
        self.xmin, self.ymin = float(x), float(y)
        self.xmax, self.ymax = float(x + w), float(y + h)

    @property
    def rect(self):
        return (self.xmin, self.ymin, self.xmax, self.ymax)


def _segment_hits_rect(p1, p2, rect):
    """Liang-Barsky 算法：判断线段与轴对齐矩形是否相交。"""
    x1, y1 = float(p1[0]), float(p1[1])
    x2, y2 = float(p2[0]), float(p2[1])
    xmin, ymin, xmax, ymax = rect
    dx, dy = x2 - x1, y2 - y1
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, x1 - xmin), (dx, xmax - x1),
                 (-dy, y1 - ymin), (dy, ymax - y1)):
        if p == 0.0:
            if q < 0.0:
                return False
        else:
            t = q / p
            if p < 0.0:
                if t > t1:
                    return False
                t0 = max(t0, t)
            else:
                if t < t0:
                    return False
                t1 = min(t1, t)
    return t0 <= t1


class Environment:
    """地图 = 障碍列表 + 起点 + 目标区域。"""

    def __init__(self, name, size=612.0):
        self.name = name
        self.size = float(size)
        self.obstacles = []
        self.start = None
        self.goal = None
        self.goal_radius = 14.0
        # 该环境演示轨迹的点数（取自论文正文）
        self.n_demo_points = 82

    def add_rect(self, x, y, w, h):
        self.obstacles.append(Obstacle(x, y, w, h))
        return self

    def is_free_point(self, p):
        """点是否在地图内且不落在任何障碍内。"""
        x, y = float(p[0]), float(p[1])
        if not (0.0 <= x <= self.size and 0.0 <= y <= self.size):
            return False
        for ob in self.obstacles:
            if ob.xmin <= x <= ob.xmax and ob.ymin <= y <= ob.ymax:
                return False
        return True

    def is_free_segment(self, p1, p2):
        """线段是否无碰撞（p1 假定已在自由空间）。"""
        if not self.is_free_point(p2):
            return False
        for ob in self.obstacles:
            if _segment_hits_rect(p1, p2, ob.rect):
                return False
        return True

    def in_goal_region(self, p):
        return bool(np.hypot(float(p[0]) - self.goal[0],
                             float(p[1]) - self.goal[1]) <= self.goal_radius)


def make_env1():
    """ENV1：三个简单障碍，起点与目标之间被挡住，必须走中间走廊。"""
    env = Environment("ENV1")
    env.start = np.array([60.0, 60.0])
    env.goal = np.array([550.0, 550.0])
    env.n_demo_points = 82
    env.add_rect(230, 170, 150, 120)     # 挡在直线路径上
    env.add_rect(230, 380, 150, 120)     # 与上面形成中间走廊
    env.add_rect(430, 60, 90, 150)
    return env


def make_env2():
    """ENV2：多障碍拥挤环境。"""
    env = Environment("ENV2")
    env.start = np.array([50.0, 50.0])
    env.goal = np.array([560.0, 560.0])
    env.n_demo_points = 129
    for x, y, w, h in [
        (150, 100, 80, 120), (280, 60, 80, 200), (420, 120, 90, 90),
        (120, 300, 200, 70), (380, 280, 70, 180), (500, 260, 80, 160),
        (220, 430, 80, 130), (350, 480, 140, 70), (520, 480, 60, 60),
        (60, 480, 90, 90), (280, 300, 60, 60),
    ]:
        env.add_rect(x, y, w, h)
    return env


def make_env3():
    """ENV3：另一组拥挤环境，障碍更密。"""
    env = Environment("ENV3")
    env.start = np.array([40.0, 570.0])
    env.goal = np.array([570.0, 40.0])
    env.n_demo_points = 124
    for x, y, w, h in [
        (100, 100, 70, 160), (100, 340, 70, 160), (100, 540, 200, 60),
        (250, 80, 90, 200), (250, 400, 90, 120), (400, 100, 70, 170),
        (400, 340, 70, 200), (520, 150, 60, 130), (520, 380, 60, 150),
        (180, 260, 120, 60), (330, 250, 60, 90),
    ]:
        env.add_rect(x, y, w, h)
    return env


ALL_ENVS = [make_env1, make_env2, make_env3]


def draw_env(ax, env):
    """把地图画到给定的 matplotlib 坐标轴上。"""
    import matplotlib.pyplot as plt
    for ob in env.obstacles:
        ax.add_patch(plt.Rectangle((ob.xmin, ob.ymin),
                                   ob.xmax - ob.xmin, ob.ymax - ob.ymin,
                                   facecolor="0.35", edgecolor="k", zorder=2))
    ax.plot(env.start[0], env.start[1], "o", color="magenta",
            markersize=9, zorder=6)
    ax.add_patch(plt.Circle(env.goal, env.goal_radius, color="green",
                            alpha=0.9, zorder=6))
    ax.set_xlim(0, env.size)
    ax.set_ylim(0, env.size)
    ax.set_aspect("equal")


if __name__ == "__main__":
    import os
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(15, 5.2))
    for ax, factory in zip(axes, ALL_ENVS):
        env = factory()
        draw_env(ax, env)
        ax.set_title(f"{env.name}   demo points N={env.n_demo_points}")
    plt.tight_layout()
    os.makedirs("结果", exist_ok=True)
    plt.savefig(os.path.join("结果", "check_maps.png"), dpi=110)
    print("已保存 结果/check_maps.png")
