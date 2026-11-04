#!/usr/bin/env python3
"""由 turtlebot3_house 的 model.sdf 生成与 Gazebo 房屋世界一致的 2D 占据地图。

把每个 <collision> 的几何(仅 box/cylinder, mesh 忽略)按 link pose * collision pose
变换到世界坐标, 在 z 区间与激光高度重叠时投影到栅格; 用墙体紧包围盒当"室内"边界,
盒内非障碍全部标自由, 盒外 pad 圈标未知, 这样得到的地图与 SLAM 出来的样子接近,
但几何与仿真世界严格一致。

用法: python3 gen_house_gt_map.py <model.sdf> <输出前缀> --spawn -2.0 -0.5
"""
import argparse
import math
import xml.etree.ElementTree as ET

import numpy as np

RES = 0.05
OCC = 0
FREE = 254
UNKNOWN = 205


def parse_pose(text, default=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
    if not text:
        return np.array(default, dtype=float)
    v = [float(x) for x in text.split()]
    while len(v) < 6:
        v.append(0.0)
    return np.array(v[:6], dtype=float)


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def compose(p1, p2):
    """p2 在 p1 坐标系下的位姿 -> 世界位姿(只保留 z 平移与 yaw)。"""
    R1 = rpy_matrix(*p1[3:6])
    t = R1 @ p2[0:3] + p1[0:3]
    R = R1 @ rpy_matrix(*p2[3:6])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([t[0], t[1], t[2], 0.0, 0.0, yaw])


# mesh 家具没法解析几何, 按已知模型给个近似占地半径(米)
MESH_RADIUS = {
    'mailbox': 0.35,
    'trash_can': 0.28,
    'table_marble': 0.75,
}
DEFAULT_MESH_RADIUS = 0.40


def collect_obstacles(sdf_path, z_lo=0.02, z_hi=0.80):
    """递归遍历 <model>/<link>/<collision>, 累乘各级 pose 得到世界位姿。"""
    root = ET.parse(sdf_path).getroot()
    obstacles = []
    mesh_count = 0

    def add_geometry(geom, world_pose):
        nonlocal mesh_count
        box = geom.find('box')
        cyl = geom.find('cylinder')
        mesh = geom.find('mesh')
        if box is not None:
            sx, sy, sz = [float(x) for x in box.findtext('size').split()]
            z0, z1 = world_pose[2] - sz / 2, world_pose[2] + sz / 2
            if z1 < z_lo or z0 > z_hi:
                return
            obstacles.append(('box', world_pose[0], world_pose[1],
                              world_pose[5], sx / 2, sy / 2))
        elif cyl is not None:
            r = float(cyl.findtext('radius'))
            ln = float(cyl.findtext('length'))
            z0, z1 = world_pose[2] - ln / 2, world_pose[2] + ln / 2
            if z1 < z_lo or z0 > z_hi:
                return
            obstacles.append(('cyl', world_pose[0], world_pose[1], r))
        elif mesh is not None:
            uri = (mesh.findtext('uri') or '').lower()
            radius = DEFAULT_MESH_RADIUS
            for key, val in MESH_RADIUS.items():
                if key in uri:
                    radius = val
                    break
            obstacles.append(('cyl', world_pose[0], world_pose[1], radius))
            mesh_count += 1

    def walk(model, parent_pose):
        model_pose = compose(parent_pose, parse_pose(model.findtext('pose')))
        for link in model.findall('link'):
            link_pose = compose(model_pose, parse_pose(link.findtext('pose')))
            for col in link.findall('collision'):
                col_pose = compose(link_pose, parse_pose(col.findtext('pose')))
                geom = col.find('geometry')
                if geom is not None:
                    add_geometry(geom, col_pose)
        for sub in model.findall('model'):
            walk(sub, model_pose)

    walk(root, np.zeros(6))
    return obstacles, mesh_count


def rasterize(obstacles, res, pad_cells=8):
    pts = []
    for ob in obstacles:
        if ob[0] == 'box':
            _, cx, cy, yaw, hx, hy = ob
            r = math.hypot(hx, hy)
        else:
            _, cx, cy, r = ob
        pts.append((cx - r, cy - r))
        pts.append((cx + r, cy + r))
    minx = min(p[0] for p in pts) - pad_cells * res
    miny = min(p[1] for p in pts) - pad_cells * res
    maxx = max(p[0] for p in pts) + pad_cells * res
    maxy = max(p[1] for p in pts) + pad_cells * res
    width = int(math.ceil((maxx - minx) / res))
    height = int(math.ceil((maxy - miny) / res))
    grid = np.full((height, width), UNKNOWN, dtype=np.uint8)

    def world_to_cell(x, y):
        col = int(math.floor((x - minx) / res))
        row = int(math.floor((maxy - y) / res))
        return row, col

    step = res / 2
    for ob in obstacles:
        if ob[0] == 'box':
            _, cx, cy, yaw, hx, hy = ob
            r = math.hypot(hx, hy)
            ca, sa = math.cos(-yaw), math.sin(-yaw)
            for y in np.arange(cy - r, cy + r + step, step):
                for x in np.arange(cx - r, cx + r + step, step):
                    dx, dy = x - cx, y - cy
                    lx = ca * dx - sa * dy
                    ly = sa * dx + ca * dy
                    if abs(lx) <= hx and abs(ly) <= hy:
                        row, col = world_to_cell(x, y)
                        if 0 <= row < height and 0 <= col < width:
                            grid[row, col] = OCC
        else:
            _, cx, cy, r = ob
            for y in np.arange(cy - r, cy + r + step, step):
                for x in np.arange(cx - r, cx + r + step, step):
                    if (x - cx) ** 2 + (y - cy) ** 2 <= r * r:
                        row, col = world_to_cell(x, y)
                        if 0 <= row < height and 0 <= col < width:
                            grid[row, col] = OCC
    return grid, minx, miny, maxy, width, height


def finalize(grid, minx, maxy, res, pad=8):
    """裁到墙体紧包围盒, 盒内非障碍标自由, 盒外 pad 圈标未知。

    房子外墙有门洞, 用泛洪填充会顺着门洞漏到房子外面, 所以直接用墙体的
    紧包围盒当室内边界。
    """
    rows, cols = np.where(grid == OCC)
    r0, r1 = int(rows.min()), int(rows.max())
    c0, c1 = int(cols.min()), int(cols.max())
    R0 = max(0, r0 - pad)
    R1 = min(grid.shape[0], r1 + pad + 1)
    C0 = max(0, c0 - pad)
    C1 = min(grid.shape[1], c1 + pad + 1)
    sub = grid[R0:R1, C0:C1].copy()
    sub[sub != OCC] = FREE
    inside = np.zeros(sub.shape, dtype=bool)
    inside[r0 - R0:r1 - R0 + 1, c0 - C0:c1 - C0 + 1] = True
    sub[~inside] = UNKNOWN
    sub[grid[R0:R1, C0:C1] == OCC] = OCC
    return sub, minx + C0 * res, maxy - R0 * res


def write_pgm(path, grid):
    h, w = grid.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(grid.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sdf')
    ap.add_argument('out_prefix')
    ap.add_argument('--spawn', nargs=2, type=float, default=(-2.0, -0.5))
    args = ap.parse_args()

    obstacles, skipped = collect_obstacles(args.sdf)
    grid, minx, miny, maxy, _w, _h = rasterize(obstacles, RES)
    grid, minx, maxy = finalize(grid, minx, maxy, RES)
    h, w = grid.shape
    miny2 = maxy - h * RES

    row = int(round((maxy - args.spawn[1]) / RES))
    col = int(round((args.spawn[0] - minx) / RES))
    if not (0 <= row < h and 0 <= col < w):
        raise SystemExit('出生点 %s 不在栅格范围内' % (args.spawn,))
    if grid[row, col] != FREE:
        raise SystemExit('出生点 %s 落在非自由格 %d 上'
                         % (args.spawn, int(grid[row, col])))

    name = args.out_prefix.replace('\\', '/').split('/')[-1]
    write_pgm(args.out_prefix + '.pgm', grid)
    with open(args.out_prefix + '.yaml', 'w', encoding='utf-8') as f:
        f.write('image: %s.pgm\n' % name)
        f.write('mode: trinary\n')
        f.write('resolution: %.4f\n' % RES)
        f.write('origin: [%.4f, %.4f, 0.0]\n' % (minx, miny2))
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.25\n')
    occ = int((grid == OCC).sum())
    free = int((grid == FREE).sum())
    print('几何: %d 个 (跳过 mesh %d 个)' % (len(obstacles), skipped))
    print('地图: %dx%d res=%.2f origin=(%.3f, %.3f)'
          % (w, h, RES, minx, miny2))
    print('占据 %d 格 (%.1f%%), 自由 %d 格 (%.1f%%), 未知 %d 格'
          % (occ, occ / (w * h) * 100, free, free / (w * h) * 100,
             w * h - occ - free))


if __name__ == '__main__':
    main()