#!/usr/bin/env python3
"""从 GT 地图里挑一套离障碍足够远、分散在大厅不同角落的航点。

用法示例:
  python3 pick_house_wps.py _vmfix/house_map/house.pgm -2.0 -0.5

输出 python 列表形式的 waypoint_x / waypoint_y, 直接贴进 launch 的 SCENES。
"""
import argparse
import math

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt


def load_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        w, h = [int(x) for x in f.readline().split()]
        maxval = int(f.readline())
        assert maxval == 255
        data = np.frombuffer(f.read(), dtype=np.uint8)
    return data.reshape(h, w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pgm')
    ap.add_argument('spawn', nargs=2, type=float)
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--origin', nargs=2, type=float, required=True)
    ap.add_argument('--clear', type=float, default=0.45)
    ap.add_argument('--min-dist', type=float, default=1.5)
    ap.add_argument('--n', type=int, default=4)
    ap.add_argument('--sep', type=float, default=2.0)
    args = ap.parse_args()

    grid = load_pgm(args.pgm)
    h, w = grid.shape
    res = args.res
    ox, oy = args.origin
    maxy = oy + h * res

    # 占据(0)与未知(205)都不能走; 254 才自由
    free = grid == 254
    d = distance_transform_edt(free) * res
    ok = free & (d >= args.clear)

    rows, cols = np.indices((h, w))
    xs = ox + (cols + 0.5) * res
    ys = maxy - (rows + 0.5) * res
    dist_from_spawn = np.hypot(xs - args.spawn[0], ys - args.spawn[1])
    ok &= dist_from_spawn >= args.min_dist

    cand = np.argwhere(ok)
    print('候选点 %d 个 (净空 >= %.2f m, 距出生点 >= %.2f m)'
          % (len(cand), args.clear, args.min_dist))
    if len(cand) == 0:
        return
    # 从出生点做一次粗栅格可达性筛选, 避免挑到隔间里
    step = 4
    coarse_free = binary_dilation(~ok, iterations=0)
    cr = int(round((maxy - args.spawn[1]) / res)) // step
    cc = int(round((args.spawn[0] - ox) / res)) // step
    ch, cw = h // step, w // step
    reach = np.zeros((ch, cw), dtype=bool)
    # 粗栅格上只看“是不是障碍/未知”, 净空只在打分时用
    blocked = (grid != 254)[::step, ::step][:ch, :cw]
    stack = [(cr, cc)]
    if 0 <= cr < ch and 0 <= cc < cw:
        reach[cr, cc] = True
    while stack:
        r, c = stack.pop()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc2 = r + dr, c + dc
            if 0 <= rr < ch and 0 <= cc2 < cw and not reach[rr, cc2] \
                    and not blocked[rr, cc2]:
                reach[rr, cc2] = True
                stack.append((rr, cc2))
    del coarse_free

    scored = []
    for r, c in cand:
        if not reach[r // step, c // step]:
            continue
        scored.append((float(d[r, c]), int(r), int(c)))
    print('可达候选 %d 个' % len(scored))
    scored.sort(reverse=True)

    picked = []
    for dd, r, c in scored:
        x = ox + (c + 0.5) * res
        y = maxy - (r + 0.5) * res
        if all(math.hypot(x - px, y - py) >= args.sep for px, py, _ in picked):
            picked.append((x, y, dd))
        if len(picked) >= args.n:
            break
    picked.sort()
    print("'waypoints_x': [%s],"
          % ', '.join('%.2f' % p[0] for p in picked))
    print("'waypoints_y': [%s],"
          % ', '.join('%.2f' % p[1] for p in picked))
    for p in picked:
        print('  (%.2f, %.2f) 净空 %.2f m' % p)


if __name__ == '__main__':
    main()