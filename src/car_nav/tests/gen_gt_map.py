"""从 worlds/obstacle_field.world 生成与仿真世界几何完全一致的占据栅格地图。

用于替代早期 SLAM 手工建图留下的漂移地图(demo2/3/4 导航时 AMCL 会跑飞)。
只做 2D 投影: 所有障碍物/墙体按底面几何投影, 高度高于 0.05m 的都算占据。
"""
import math
import os
import re

import numpy as np

RES = 0.05
ORIGIN = (-6.11, -5.95)
W, H = 242, 241            # 12.10 x 12.05 m, 与原地图一致

txt = open('worlds/obstacle_field.world', encoding='utf-8').read()
blocks = re.findall(r'<model name="([^"]+)">(.*?)</model>', txt, re.S)
print(f'model 数: {len(blocks)}')

occ = np.zeros((H, W), dtype=bool)

ys = np.linspace(0, H - 1, H)
xs = np.linspace(0, W - 1, W)
col, row = np.meshgrid(xs, ys)
gx = ORIGIN[0] + (col + 0.5) * RES          # 栅格中心世界坐标
gy = ORIGIN[1] + (H - 1 - row + 0.5) * RES  # pgm 第 0 行是最高 y

for name, body in blocks:
    if name == 'sun' or 'ground_plane' in name:
        continue
    pose = re.search(r'<pose>\s*([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)'
                     r'\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)\s*</pose>', body)
    if not pose:
        continue
    px, py, pz, _, _, psz = (float(v) for v in pose.groups())
    box = re.search(r'<box><size>\s*([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)'
                    r'\s*</size></box>', body)
    cyl = re.search(r'<cylinder>\s*<radius>\s*([-\d.eE]+)\s*</radius>', body)

    if box:
        sx, sy, sz = (float(v) for v in box.groups())
        dx = gx - px
        dy = gy - py
        c, s = math.cos(-psz), math.sin(-psz)
        lx = dx * c - dy * s
        ly = dx * s + dy * c
        hit = (np.abs(lx) <= sx / 2) & (np.abs(ly) <= sy / 2)
        hgt = sz
    elif cyl:
        r = float(cyl.group(1))
        hit = (gx - px) ** 2 + (gy - py) ** 2 <= r ** 2
        hgt = float(re.search(r'<length>\s*([-\d.eE]+)', body).group(1))
    else:
        print(f'  跳过 {name}: 未识别几何')
        continue

    if pz - hgt / 2 > 0.06:      # 完全悬空, 激光打不到
        print(f'  跳过 {name}: 悬空')
        continue
    occ |= hit
    print(f'  {name:<12} 占据格 {int(hit.sum()):5d}')

# 房间内部(四墙内)标为自由, 墙外标未知
inside = (np.abs(gx) < 5.9) & (np.abs(gy) < 5.9)
img = np.full((H, W), 205, dtype=np.uint8)     # 未知
img[inside] = 254                              # 自由
img[occ] = 0                                   # 占据

os.makedirs('_vmfix/gt_map', exist_ok=True)
with open('_vmfix/gt_map/obstacle_field.pgm', 'wb') as f:
    f.write(b'P5\n# CARLA-free ground truth map from obstacle_field.world\n')
    f.write(f'{W} {H}\n255\n'.encode())
    f.write(img.tobytes())

with open('_vmfix/gt_map/obstacle_field.yaml', 'w', encoding='utf-8') as f:
    f.write('image: obstacle_field.pgm\n'
            f'resolution: {RES}\n'
            f'origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.196\n')

print(f'\n占据 {int(occ.sum())} 格, 自由 {int((img == 254).sum())} 格')
print(f'世界包围盒 x[{gx.min():.2f},{gx.max():.2f}] y[{gy.min():.2f},{gy.max():.2f}]')
print('输出: _vmfix/gt_map/obstacle_field.pgm + .yaml')