#!/usr/bin/env python3
"""计算固定仿真场地的参考自由空间覆盖率。

参考地图只用于评估和自动停止，不提供给探索或导航节点：
  - 分母：参考地图 exploration_boundary 内的真实自由格
  - 分子：这些自由格在 SLAM 地图中已经观测为 free 的数量
  - SLAM 地图中的 unknown、occupied 或尚未扩边到的区域都算未覆盖

这样不会把墙体、柱子内部等不可达未知格计入分母，也不会把遮挡后
仍未探索的真实自由空间错误排除。
"""

import argparse
import sys

import numpy as np
from PIL import Image


def load_pgm_with_meta(pgm_path, yaml_path):
    """读 pgm 图 + yaml 元数据（origin, resolution）。"""
    img = np.array(Image.open(pgm_path))
    meta = {}
    with open(yaml_path) as f:
        for line in f:
            if ':' not in line:
                continue
            k, v = line.split(':', 1)
            k = k.strip()
            v = v.strip()
            if k == 'resolution':
                meta['resolution'] = float(v)
            elif k == 'origin':
                meta['origin'] = [float(s) for s in v.strip('[]').split(',')]
    return img, meta


def classify(img):
    """pgm 像素: 0=黑(障碍), 205=灰(未知), 254=白(自由)。"""
    free = img >= 250
    unknown = (img >= 190) & (img < 250)
    occupied = img < 190
    return free, occupied, unknown


def reference_coverage(pgm_path, yaml_path, reference_pgm, reference_yaml,
                       boundary_half=5.8):
    img, meta = load_pgm_with_meta(pgm_path, yaml_path)
    ref, ref_meta = load_pgm_with_meta(reference_pgm, reference_yaml)
    h, w = img.shape
    ref_h, ref_w = ref.shape
    res = meta['resolution']
    ref_res = ref_meta['resolution']
    ox, oy = meta['origin'][:2]
    ref_ox, ref_oy = ref_meta['origin'][:2]

    # 使用参考地图每个真实自由格中心的世界坐标，映射到 SLAM 地图。
    ref_rows, ref_cols = np.nonzero(ref >= 250)
    xs = ref_ox + (ref_cols + 0.5) * ref_res
    ys = ref_oy + (ref_h - 1 - ref_rows + 0.5) * ref_res
    in_boundary = (np.abs(xs) <= boundary_half) & (np.abs(ys) <= boundary_half)
    xs = xs[in_boundary]
    ys = ys[in_boundary]

    obs_cols = np.floor((xs - ox) / res).astype(int)
    obs_rows = np.floor((oy + h * res - ys) / res).astype(int)
    inside = ((obs_cols >= 0) & (obs_cols < w) &
              (obs_rows >= 0) & (obs_rows < h))
    observed = np.full(xs.shape, -1, dtype=np.int16)
    observed[inside] = img[obs_rows[inside], obs_cols[inside]]

    covered_free = int((observed >= 250).sum())
    remaining_unknown = int(((observed >= 190) & (observed < 250)).sum())
    remaining_occupied = int(((observed >= 0) & (observed < 190)).sum())
    outside_map = int((observed < 0).sum())
    reference_free = int(xs.size)
    coverage = covered_free / max(1, reference_free)

    free, _, unknown = classify(img)
    free_total = int(free.sum())
    unknown_total = int(unknown.sum())
    naive_cov = free_total / max(1, free_total + unknown_total)

    print(f'SLAM 地图: {w}x{h} @ {res} m/px  origin=({ox}, {oy})')
    print(f'参考地图: {ref_w}x{ref_h} @ {ref_res} m/px  '
          f'origin=({ref_ox}, {ref_oy})')
    print(f'边界: ±{boundary_half} m')
    print(f'[naive] 全图自由格 {free_total}, 全图未知格 {unknown_total}, '
          f'覆盖 {naive_cov * 100:.2f}%（仅对照，不作停止判据）')
    print(f'[reference] 界内真实自由格 {reference_free}, 已覆盖 {covered_free}, '
          f'仍未知 {remaining_unknown}, 误占 {remaining_occupied}, '
          f'图外 {outside_map}')
    print(f'[reference] 自由空间覆盖率 = {covered_free}/{reference_free} '
          f'= {coverage * 100:.2f}%')
    return coverage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pgm', nargs='?', default='obstacle_field_roadmap.pgm')
    parser.add_argument('boundary_half', nargs='?', type=float, default=5.8)
    parser.add_argument('--reference', required=True,
                        help='同一场景参考地图 PGM；对应 YAML 需在同目录')
    parser.add_argument(
        '--target', type=float,
        help='完成阈值（0~1）；达到返回 0，未达到返回 1，读取失败返回 2')
    args = parser.parse_args()

    yaml_path = args.pgm.replace('.pgm', '.yaml')
    reference_yaml = args.reference.replace('.pgm', '.yaml')
    try:
        coverage = reference_coverage(
            args.pgm, yaml_path, args.reference, reference_yaml,
            args.boundary_half)
    except (OSError, ValueError, KeyError, IndexError) as exc:
        print(f'覆盖率计算失败: {exc}', file=sys.stderr)
        return 2

    if args.target is not None:
        print(f'[target] 当前 {coverage * 100:.2f}% / 目标 {args.target * 100:.2f}%')
        return 0 if coverage >= args.target else 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
