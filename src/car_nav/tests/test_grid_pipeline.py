#!/usr/bin/env python3
"""离线全链路测试: 用已保存的 SLAM 地图跑一遍 GMR-RRT* 完整流程。

不依赖 ROS 运行时(只依赖 gmr_lib + grid_env + numpy/sklearn),
用于在接入 Nav2 之前验证适配层正确性。

用法(在 car_ws 下):
  python3 src/car_nav/tests/test_grid_pipeline.py [地图pgm] [分辨率] [origin_x] [origin_y]
"""

import os
import sys
import time

import numpy as np


def read_pgm(path):
    """读 P5 二进制 pgm -> (2D uint8 数组, width, height)。行 0 = 图像顶部。"""
    with open(path, 'rb') as f:
        data = f.read()
    # 解析头: P5 <w> <h> <max> <binary...>, 跳过注释
    pos = 0
    fields = []
    while len(fields) < 4:
        # 跳过空白与注释
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while data[pos:pos + 1] not in (b'\n', b''):
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        fields.append(data[start:pos])
    pos += 1      # 头部最后一个空白符
    magic, w, h, _max = fields
    assert magic == b'P5', f'不是二进制 pgm: {magic}'
    w, h = int(w), int(h)
    img = np.frombuffer(data[pos:pos + w * h], dtype=np.uint8).reshape(h, w)
    return img, w, h


def main():
    pgm = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'maps', 'obstacle_field.pgm')
    res = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
    ox = float(sys.argv[3]) if len(sys.argv) > 3 else -6.11
    oy = float(sys.argv[4]) if len(sys.argv) > 4 else -5.95

    img, w, h = read_pgm(pgm)
    # pgm 行 0 是图像顶部(y 最大), OccupancyGrid 行 0 是 y 最小 -> 上下翻转
    img = img[::-1]
    # pgm: 0=占据, 205=未知, 254=自由 -> OccupancyGrid 语义
    occ = np.where(img == 0, 100, np.where(img == 205, -1, 0)).astype(np.int16)

    from car_nav.grid_env import (GridEnvironment, extract_path,
                                  generate_grid_demos)
    from car_nav.gmr_lib.gmr import fit_gmm, gmr_predict, GMRSampler
    from car_nav.gmr_lib.rrt_star import RRTStar

    env = GridEnvironment('offline_test', occ, w, h, res,
                          inflation_radius=0.18)
    print(f'地图 {w}x{h} res={res}, 可通行占比 {env._free.mean():.0%}, '
          f'env.size={env.size:.1f} m')

    # 起点/终点(世界坐标 -> env 坐标): 障碍场对角
    env.start = env.nearest_free(np.array([-4.5 - ox, -4.5 - oy]))
    env.goal = env.nearest_free(np.array([4.0 - ox, 4.0 - oy]))
    env.goal_radius = 0.25
    print(f'起点 env=({env.start[0]:.1f},{env.start[1]:.1f}) '
          f'终点 env=({env.goal[0]:.1f},{env.goal[1]:.1f}), '
          f'直线距离 {np.linalg.norm(env.goal - env.start):.1f} m')

    t0 = time.perf_counter()
    demos, ref_len = generate_grid_demos(env, n_demos=10, seed=0)
    print(f'[1/3] demos: {len(demos)} 条 x {demos[0].shape[0]} 点, '
          f'A* 参考 {ref_len:.1f} m, 耗时 {time.perf_counter() - t0:.2f}s')

    gmm, n = fit_gmm(demos, n_components=8, seed=0)
    mu, sigma = gmr_predict(gmm, n)
    print(f'[2/3] GMM+GMR: K={gmm.n_components}, 耗时 '
          f'{time.perf_counter() - t0:.2f}s 累计')

    rng = np.random.default_rng(0)
    sampler = GMRSampler(mu, sigma, env, rng)
    planner = RRTStar(env, sampler, step_size=0.30, rewire_radius=1.2,
                      max_iter=4000)
    res_dict = planner.plan()
    print(f'[3/3] GMR-RRT*: 首次成功迭代 {res_dict["iter_init"]}, '
          f'节点 {res_dict["n_nodes"]}, 路径长 {res_dict["cost_final"]:.1f} m, '
          f'耗时 {time.perf_counter() - t0:.2f}s')

    path = extract_path(res_dict, env)
    assert path is not None, '未找到路径!'
    # 校验: 路径逐段无碰撞
    ok = all(env.is_free_segment(path[i], path[i + 1])
             for i in range(len(path) - 1))
    print(f'路径 {len(path)} 个点, 无碰撞校验: {"通过" if ok else "失败!"}')
    print('=== 离线全链路测试通过 ===' if ok else '!!! 存在问题 !!!')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
