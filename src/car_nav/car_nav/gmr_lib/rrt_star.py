"""RRT* 骨架（算法主干可复用，采样器可插拔）。

对应论文 Algorithm 1 主干。三个算法（RRT* / GMM-RRT* / GMR-RRT*）
共用同一份实现，只替换 sampler —— 保证对比公平。

论文参数：step size = 6，rewire radius = 35，goal radius = 14。
"""

import time
import numpy as np

STEP_SIZE = 6.0
REWIRE_RADIUS = 35.0
MAX_ITER = 2000


class RRTStar:
    def __init__(self, env, sampler, step_size=STEP_SIZE,
                 rewire_radius=REWIRE_RADIUS, max_iter=MAX_ITER, seed=0):
        self.env = env
        self.sampler = sampler
        self.step = float(step_size)
        self.rewire_r = float(rewire_radius)
        self.max_iter = int(max_iter)
        self.seed = int(seed)

    # ------------------------------------------------------------ 主循环
    def plan(self):
        env, step, rw = self.env, self.step, self.rewire_r
        cap = self.max_iter + 2

        pts = np.zeros((cap, 2))
        par = np.full(cap, -1, dtype=int)
        cost = np.zeros(cap)
        children = [[] for _ in range(cap)]

        pts[0] = env.start
        n = 1
        best_cost = np.inf
        goal_parent = -1
        cost_curve = np.full(self.max_iter, np.inf)
        nodes_curve = np.zeros(self.max_iter, dtype=int)

        iter_init = -1
        time_init = np.nan
        cost_init = np.nan

        t0 = time.perf_counter()

        for it in range(self.max_iter):
            q = np.asarray(self.sampler.sample(), dtype=float)

            # 1. 最近邻
            i_near = int(np.argmin(((pts[:n] - q) ** 2).sum(axis=1)))

            # 2. steer：朝采样点走固定步长
            vec = q - pts[i_near]
            dist = float(np.linalg.norm(vec))
            if dist < 1e-9:
                cost_curve[it] = best_cost
                nodes_curve[it] = n
                continue
            q_new = q if dist <= step else pts[i_near] + vec * (step / dist)

            # 3. 碰撞检测
            if not env.is_free_segment(pts[i_near], q_new):
                cost_curve[it] = best_cost
                nodes_curve[it] = n
                continue

            # 4. 半径内的邻居（用于 choose-parent 与 rewire）
            d2 = ((pts[:n] - q_new) ** 2).sum(axis=1)
            near = np.where(d2 <= rw * rw)[0]

            # 5. choose parent
            best_p = i_near
            best_c = cost[i_near] + float(np.linalg.norm(q_new - pts[i_near]))
            for j in near:
                if not env.is_free_segment(pts[j], q_new):
                    continue
                c = cost[j] + float(np.linalg.norm(q_new - pts[j]))
                if c < best_c:
                    best_c, best_p = c, int(j)

            pts[n] = q_new
            par[n] = best_p
            cost[n] = best_c
            children[best_p].append(n)
            i_new = n
            n += 1

            # 6. rewire
            for j in near:
                j = int(j)
                if j == best_p:
                    continue
                c = best_c + float(np.linalg.norm(q_new - pts[j]))
                if c >= cost[j] - 1e-9:
                    continue
                if self._is_ancestor(j, i_new, par):
                    continue        # 防止产生环
                if not env.is_free_segment(q_new, pts[j]):
                    continue
                children[par[j]].remove(j)
                par[j] = i_new
                children[i_new].append(j)
                cost[j] = c
                # 子树代价同步更新
                stack = list(children[j])
                while stack:
                    u = stack.pop()
                    cost[u] = cost[par[u]] + float(np.linalg.norm(pts[u] - pts[par[u]]))
                    stack.extend(children[u])

            # 7. 触及目标区域
            if env.in_goal_region(q_new) and env.is_free_segment(q_new, env.goal):
                c = cost[i_new] + float(np.linalg.norm(env.goal - q_new))
                if c < best_cost:
                    best_cost = c
                    goal_parent = i_new
                    if iter_init < 0:
                        iter_init = it + 1
                        time_init = time.perf_counter() - t0
                        cost_init = c

            cost_curve[it] = best_cost
            nodes_curve[it] = n

        total_time = time.perf_counter() - t0

        # 收敛迭代数：首次进入最终最优解的 1% 以内
        n_nodes_init = -1
        n_nodes_conv = -1
        if np.isfinite(best_cost):
            idx = np.where(cost_curve <= best_cost * 1.01)[0]
            iter_conv = int(idx[0]) + 1 if len(idx) else self.max_iter
            n_nodes_conv = int(nodes_curve[iter_conv - 1])
        else:
            iter_conv = -1
        if iter_init > 0:
            n_nodes_init = int(nodes_curve[iter_init - 1])

        return {
            "algo": getattr(self.sampler, "name", "?"),
            "n_nodes": n,
            "iter_init": iter_init,
            "time_init": time_init,
            "cost_init": cost_init,
            "cost_final": best_cost,
            "iter_conv": iter_conv,
            "time_total": total_time,
            "cost_curve": cost_curve,
            "nodes_curve": nodes_curve,
            "n_nodes_init": n_nodes_init,
            "n_nodes_conv": n_nodes_conv,
            "goal_parent": goal_parent,
            "pts": pts[:n].copy(),
            "par": par[:n].copy(),
        }

    # ------------------------------------------------------------ 工具
    @staticmethod
    def _is_ancestor(a, b, par):
        """a 是否为 b 的祖先（含自身）。"""
        u = b
        while u != -1:
            if u == a:
                return True
            u = par[u]
            if u < 0:
                break
        return False
