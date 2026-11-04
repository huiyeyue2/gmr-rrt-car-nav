#!/usr/bin/env python3
"""从已保存的 SLAM 地图中自动挑选巡逻航点。

策略: 每 0.8m 网格采样候选点 -> 过滤(自由 + 离障碍/未知 0.4m 净空 + 与出生点连通)
      -> 贪心最大-最小间距选 n 个(保证分散) -> 输出 world 坐标。

用法(在 car_ws/src/car_nav 下):
  python3 tests/pick_waypoints.py [pgm] [分辨率] [origin_x] [origin_y] [n]
"""

import sys

import numpy as np

sys.path.insert(0, 'tests')
from test_grid_pipeline import read_pgm          # noqa: E402

from car_nav.grid_env import GridEnvironment     # noqa: E402
from car_nav.gmr_lib.demos import _astar         # noqa: E402


def main():
    pgm = sys.argv[1] if len(sys.argv) > 1 else 'maps/house.pgm'
    res = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
    ox = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    oy = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    n_wp = int(sys.argv[5]) if len(sys.argv) > 5 else 5
    spawn_wx = float(sys.argv[6]) if len(sys.argv) > 6 else -2.0
    spawn_wy = float(sys.argv[7]) if len(sys.argv) > 7 else -0.5

    img, w, h = read_pgm(pgm)
    img = img[::-1]                                  # pgm 顶部 -> OccupancyGrid 底部
    occ = np.where(img == 0, 100, np.where(img == 205, -1, 0)).astype(np.int16)

    env = GridEnvironment('wp', occ, w, h, res, inflation_radius=0.22)
    print(f'地图 {w}x{h} res={res} origin=({ox},{oy}), '
          f'自由占比 {env._free.mean():.0%}')

    # 出生点(默认 -2.0, -0.5; 障碍场为 -4.5, -4.5)
    spawn = env.nearest_free(np.array([spawn_wx - ox, spawn_wy - oy]))
    if spawn is None:
        raise SystemExit('出生点附近无自由栅格?!')

    # 连通性: 从出生点在粗栅格上 BFS 标记可达区域
    free, (nh, nw) = env.coarse_grid(0.15)
    sc = (int(spawn[1] / 0.15), int(spawn[0] / 0.15))     # (row, col)
    sc = (min(sc[0], nh - 1), min(sc[1], nw - 1))
    reach = np.zeros_like(free)
    stack = [sc]
    reach[sc] = True
    while stack:
        r, c = stack.pop()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < nh and 0 <= nc < nw and free[nr, nc] \
                    and not reach[nr, nc]:
                reach[nr, nc] = True
                stack.append((nr, nc))
    print(f'从出生点可达的粗栅格: {reach.mean():.0%}')

    # 候选点: 0.8m 网格上取自由点(带 0.4m 净空 = 0.22 膨胀 + 再留 margin)
    cands = []
    step = 0.8
    for gy in np.arange(0.5, env.size, step):
        for gx in np.arange(0.5, env.size, step):
            p = np.array([gx, gy])
            if not env.is_free_point(p):
                continue
            # 额外净空检查: 0.4m 半径内全部自由(允许穿过未知, 只挡占据)
            cell = env._to_cell(gx, gy)
            r0, c0 = cell
            rr = int(0.4 / res)
            if r0 - rr < 0 or c0 - rr < 0 or \
                    r0 + rr >= env.height or c0 + rr >= env.width:
                continue
            win = env._free[r0 - rr:r0 + rr, c0 - rr:c0 + rr]
            if not win.all():
                continue
            # 可达
            cc = (int(gy / 0.15), int(gx / 0.15))
            if not reach[min(cc[0], nh - 1), min(cc[1], nw - 1)]:
                continue
            cands.append(p)
    print(f'候选点: {len(cands)} 个')

    # 贪心最大-最小间距选 n 个(第一个选离出生点 1.5~4m 的)
    wps = []
    best_first = max(
        (c for c in cands
         if 1.5 < np.linalg.norm(c - spawn) < 4.0),
        key=lambda c: np.linalg.norm(c - spawn), default=None)
    if best_first is None and cands:
        best_first = cands[len(cands) // 2]
    wps.append(best_first)
    while len(wps) < n_wp and cands:
        wps.append(max(cands, key=lambda c: min(
            np.linalg.norm(c - w) for w in wps)))

    print('\n=== 巡逻航点(world 坐标, 可直接写入 mission_runner) ===')
    print('waypoint_x:=[' + ', '.join(f'{w[0] + ox:.2f}' for w in wps) + ']')
    print('waypoint_y:=[' + ', '.join(f'{w[1] + oy:.2f}' for w in wps) + ']')
    for i, w in enumerate(wps):
        print(f'WP{i + 1}: world ({w[0] + ox:.2f}, {w[1] + oy:.2f})  '
              f'距出生点 {np.linalg.norm(w - spawn):.1f} m')


if __name__ == '__main__':
    main()
