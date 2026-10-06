#!/usr/bin/env python3
"""栅格地图质量评估：把地图和 world 的真实几何逐格比对。

    ros2 run my_robot_description mapeval.py <map.yaml> [--world rooms.sdf] [--json]

评估内容
--------
* 地图尺寸 / 占用 / 空闲 / 未知 占比
* 每个占用格到**最近真实墙面**的距离（均值/中位/最大/各阈值通过率）
* 幻影墙：整块离真实墙面 >15 cm 的连通分量（地图上凭空多出来的东西）
* 可见墙面覆盖率：沿真实墙面采样，10 cm 内有没有占用格
* 门洞通畅度：每个门洞里有多少占用格（理想是 0）
* 障碍物进图情况：名字以 `obs_` 开头的 box 单独统计——离它最近的占用格
  有多远（理想 ≈0）、它的边界被覆盖了多少
* 真实墙段对照：同样方法量一段确实是墙的地方，作为"该有多少"的参照
* 整体平移：地图相对 world 的最佳对齐偏移量

墙面与门洞**从 SDF 自动解析**（只认 box 几何，ground_plane 那种 plane 会被
忽略），所以换 world 不用改这个脚本。门洞由"同一堵墙上共线的两段 box 之间的
空隙"推出。

障碍物与结构墙靠**名字前缀**区分：`obs_*` 算障碍物，其余算结构墙。
门洞推导、幻影块判定、对照墙段只用结构墙（不然障碍物之间、障碍物与墙之间
的空隙会被误判成门洞）；而"占用格到最近几何体"的距离用**全部** box，
因为障碍物在图上本来就该有占用格。
"""
import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np


# --------------------------------------------------------------------------
# world 几何
# --------------------------------------------------------------------------
def _floats(text, n=None):
    # SDF 的 pose 用空格分隔，map yaml 的 origin 用逗号（还带方括号），都吃
    vals = [float(v) for v in (text or '').replace(',', ' ').split()]
    if n is not None:
        while len(vals) < n:
            vals.append(0.0)
    return vals


def parse_world(sdf_path, min_size=0.02, max_size=50.0):
    """从 SDF 里抽出所有 box 碰撞体，返回 [(name, xmin, xmax, ymin, ymax), ...]。

    只认 <collision> 里的 <box>；<plane>（地面）自动跳过。
    模型 pose 支持绕 z 旋转（其余角度按 AABB 处理并给出警告）。
    """
    root = ET.parse(sdf_path).getroot()
    walls = []
    for model in root.iter('model'):
        name = model.get('name', '?')
        pose = _floats(model.findtext('pose'), 6)
        mx, my = pose[0], pose[1]
        yaw = pose[5]
        # 只认 <collision>/<geometry>/<box>；<plane>（地面）自然被跳过
        for box in model.iter('box'):
            size_el = box.find('size')
            if size_el is None:
                continue
            sx, sy, _ = _floats(size_el.text, 3)
            if not (min_size <= max(sx, sy) <= max_size):
                continue
            if abs(yaw) < 1e-6:
                xs = (mx - sx / 2, mx + sx / 2)
                ys = (my - sy / 2, my + sy / 2)
            else:
                c, s = math.cos(yaw), math.sin(yaw)
                pts = []
                for dx in (-sx / 2, sx / 2):
                    for dy in (-sy / 2, sy / 2):
                        pts.append((mx + dx * c - dy * s, my + dx * s + dy * c))
                xs = (min(p[0] for p in pts), max(p[0] for p in pts))
                ys = (min(p[1] for p in pts), max(p[1] for p in pts))
            walls.append((name, xs[0], xs[1], ys[0], ys[1]))
            break
    if not walls:
        raise SystemExit(f'{sdf_path}: 没找到任何 box 碰撞体，无法评估')
    return walls


def derive_doors(walls, min_gap=0.3, tol=0.06):
    """由共线墙段之间的空隙推出门洞。

    返回 [(label, axis, span_lo, span_hi, perp_lo, perp_hi), ...]
    axis='x' 表示门沿 x 方向张开（门所在的那堵墙是水平的）。
    """
    doors = []
    for axis, key_i in (('x', 3), ('y', 1)):       # x: 按 y 分组; y: 按 x 分组
        groups = {}
        for w in walls:
            _, x0, x1, y0, y1 = w
            dx, dy = x1 - x0, y1 - y0
            if axis == 'x' and dx < dy:
                continue
            if axis == 'y' and dy < dx:
                continue
            perp = (y0 + y1) / 2 if axis == 'x' else (x0 + x1) / 2
            groups.setdefault(round(perp / tol), []).append(w)
        for _, segs in sorted(groups.items()):
            segs.sort(key=lambda w: w[1] if axis == 'x' else w[3])
            for a, b in zip(segs, segs[1:]):
                lo = a[2] if axis == 'x' else a[4]
                hi = b[1] if axis == 'x' else b[3]
                if hi - lo < min_gap:
                    continue
                p0 = min(a[key_i], b[key_i])
                p1 = max(a[key_i + 1], b[key_i + 1])
                doors.append((f'门 X∈[{lo:+.2f},{hi:+.2f}]' if axis == 'x'
                              else f'门 Y∈[{lo:+.2f},{hi:+.2f}]',
                              axis, lo, hi, p0, p1))
    return doors


def longest_wall_segment(walls, length=1.0):
    """挑最长的一面墙，取中间 length 米作为"确实是墙"的对照段。"""
    w = max(walls, key=lambda w: max(w[2] - w[1], w[4] - w[3]))
    _, x0, x1, y0, y1 = w
    if (x1 - x0) >= (y1 - y0):
        mid = (x0 + x1) / 2
        seg = (mid - length / 2, mid + length / 2, y0, y1)
    else:
        mid = (y0 + y1) / 2
        seg = (x0, x1, mid - length / 2, mid + length / 2)
    return w[0], seg


# --------------------------------------------------------------------------
# 地图读写
# --------------------------------------------------------------------------
def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    toks, i = [], 0
    while len(toks) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        toks.append(data[i:j])
        i = j
    if toks[0] != b'P5':
        raise SystemExit(f'{path}: 只支持 P5 二进制 pgm，实际是 {toks[0]!r}')
    w, h = int(toks[1]), int(toks[2])
    i += 1
    px = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)
    return px


def load_map(yaml_path):
    d = {}
    with open(yaml_path) as f:
        for line in f:
            line = line.split('#')[0].strip()
            if ':' in line:
                k, v = line.split(':', 1)
                d[k.strip()] = v.strip()
    res = float(d['resolution'])
    org = _floats(d['origin'].strip('[]'), 3)
    img = d.get('image') or os.path.splitext(yaml_path)[0] + '.pgm'
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img)
    if not os.path.isfile(img):
        raise SystemExit(f'找不到地图图像: {img}')
    px = read_pgm(img)
    h, w = px.shape
    return dict(res=res, ox=org[0], oy=org[1], h=h, w=w, px=px,
                occ=(px < 100), unk=((px >= 100) & (px < 250)), free=(px >= 250))


def cell_centers(m):
    res, ox, oy, h, w = m['res'], m['ox'], m['oy'], m['h'], m['w']
    cols = ox + (np.arange(w) + 0.5) * res
    rows = oy + (h - 1 - np.arange(h) + 0.5) * res   # 图像第 0 行是 y 最大
    return np.meshgrid(cols, rows)


def dist_to_walls(X, Y, walls):
    d = np.full(X.shape, np.inf)
    for _, x0, x1, y0, y1 in walls:
        dx = np.maximum(np.maximum(x0 - X, X - x1), 0.0)
        dy = np.maximum(np.maximum(y0 - Y, Y - y1), 0.0)
        d = np.minimum(d, np.hypot(dx, dy))
    return d


def components(occ):
    h, w = occ.shape
    lab = np.zeros((h, w), np.int32)
    cur = 0
    for sy in range(h):
        for sx in range(w):
            if not occ[sy, sx] or lab[sy, sx]:
                continue
            cur += 1
            stack = [(sy, sx)]
            lab[sy, sx] = cur
            while stack:
                y, x = stack.pop()
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < h and 0 <= nx < w and occ[ny, nx] and not lab[ny, nx]:
                        lab[ny, nx] = cur
                        stack.append((ny, nx))
    return lab, cur


def band_occupancy(m, axis, lo, hi, p0, p1, exclude=()):
    """在 [lo,hi] x [p0,p1] 这个矩形里数占用格，返回 (占用数, 总格数, 其中落在 exclude 里的)。

    ★ 按**格心**遍历，不是按固定步长扫。按步长扫的话，采样点落在地图网格
    之外会被静默丢掉，于是同一个物理区域在不同地图上采到的格数不一样
    （实测过：同一段 1 m 墙，两张图一个采到 3 行、一个只采到 1 行），
    跨地图就没法比了。

    exclude 是一组 (name, x0, x1, y0, y1)。门洞里本来就有障碍物的 world
    （比如 rooms_obstacles.sdf 里被柱子塞住的门AB），那些占用格是**应该**
    有的，不减掉就会把"柱子画对了"误判成"门被墙封死了"。
    """
    res, ox, oy, h, w = m['res'], m['ox'], m['oy'], m['h'], m['w']
    oy_max = oy + h * res
    if axis == 'y':                      # 竖直墙：交换 x/y
        lo, hi, p0, p1 = p0, p1, lo, hi
    c0 = int(np.floor((lo - ox) / res))
    c1 = int(np.ceil((hi - ox) / res)) + 1
    r0 = int(np.floor((oy_max - p1) / res))
    r1 = int(np.ceil((oy_max - p0) / res)) + 1
    n = tot = nex = 0
    for r in range(max(0, r0), min(h, r1)):
        cy = oy + (h - 1 - r + 0.5) * res
        if not (p0 <= cy <= p1):
            continue
        for c in range(max(0, c0), min(w, c1)):
            cx = ox + (c + 0.5) * res
            if lo <= cx <= hi:
                tot += 1
                if m['occ'][r, c]:
                    n += 1
                    if any(x0 <= cx <= x1 and y0 <= cy <= y1
                           for _, x0, x1, y0, y1 in exclude):
                        nex += 1
    return n, tot, nex


def coverage_of_segments(m, segs, radius=0.10):
    """沿给定线段采样，看附近 radius 内有没有占用格。返回 (命中, 总数)。

    segs 是 [(x0, x1, y0, y1), ...]，每个元素是一条轴对齐线段。
    """
    res, ox, oy, h, w = m['res'], m['ox'], m['oy'], m['h'], m['w']
    occ = m['occ']
    oy_max = oy + h * res
    total = hit = 0
    for x0, x1, y0, y1 in segs:
        n = max(2, int(max(x1 - x0, y1 - y0) / (res / 2.0)))
        for i in range(n + 1):
            t = i / n
            px, py = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            total += 1
            c0, c1 = int((px - radius - ox) / res), int((px + radius - ox) / res) + 1
            r0 = int((oy_max - (py + radius)) / res)
            r1 = int((oy_max - (py - radius)) / res) + 1
            found = False
            for rr in range(max(0, r0), min(h, r1 + 1)):
                for cc in range(max(0, c0), min(w, c1 + 1)):
                    if occ[rr, cc]:
                        cx = ox + (cc + 0.5) * res
                        cy = oy + (h - 1 - rr + 0.5) * res
                        if math.hypot(cx - px, cy - py) <= radius:
                            found = True
                            break
                if found:
                    break
            hit += found
    return hit, total


def wall_coverage(m, walls, radius=0.10):
    return coverage_of_segments(m, [(x0, x1, y0, y1) for _, x0, x1, y0, y1 in walls],
                               radius)


def best_shift(X, Y, occ, walls, span=0.10, step=0.01):
    best = (float('inf'), 0.0, 0.0)
    for dx in np.arange(-span, span + 1e-9, step):
        for dy in np.arange(-span, span + 1e-9, step):
            d = dist_to_walls(X[occ] + dx, Y[occ] + dy, walls)
            if d.mean() < best[0]:
                best = (d.mean(), dx, dy)
    return best


# --------------------------------------------------------------------------
def evaluate(map_yaml, world_sdf, do_shift=True):
    m = load_map(map_yaml)
    solids = parse_world(world_sdf)
    walls = [w for w in solids if not w[0].startswith('obs_')]
    obstacles = [w for w in solids if w[0].startswith('obs_')]
    if not walls:
        raise SystemExit(f'{world_sdf}: 除障碍物外没有任何结构墙，没法评估')
    doors = derive_doors(walls)
    X, Y = cell_centers(m)
    D = dist_to_walls(X, Y, solids)      # 距离用全部几何体：障碍物在图上也是占用格
    occ, unk, free = m['occ'], m['unk'], m['free']
    tot = m['h'] * m['w']

    do = D[occ]
    lab, ncomp = components(occ)
    phantom = [c for c in range(1, ncomp + 1) if D[lab == c].min() > 0.15]
    big = [c for c in phantom if (lab == c).sum() >= 4]
    hit, wtot = wall_coverage(m, walls)

    out = {
        'map': os.path.abspath(map_yaml),
        'world': os.path.abspath(world_sdf),
        'width': m['w'], 'height': m['h'], 'resolution': m['res'],
        'size_m': [m['w'] * m['res'], m['h'] * m['res']],
        'origin': [m['ox'], m['oy']],
        'occupied': int(occ.sum()), 'free': int(free.sum()), 'unknown': int(unk.sum()),
        'occupied_pct': 100.0 * occ.sum() / tot,
        'dist_cm': {'mean': 100 * do.mean(), 'median': 100 * float(np.median(do)),
                    'max': 100 * do.max(),
                    'le5': 100 * float((do <= 0.05).mean()),
                    'le10': 100 * float((do <= 0.10).mean()),
                    'le15': 100 * float((do <= 0.15).mean())},
        'components': ncomp, 'phantom_blocks': len(phantom), 'phantom_big': len(big),
        'coverage_pct': 100.0 * hit / wtot, 'coverage': [hit, wtot],
        'doors': [], 'obstacles': [], 'control': None,
    }
    for label, axis, lo, hi, p0, p1 in doors:
        n, t, nex = band_occupancy(m, axis, lo, hi, p0, p1, obstacles)
        out['doors'].append({'label': label, 'occupied': n, 'cells': t,
                             'occupied_in_obstacle': nex, 'occupied_net': n - nex,
                             'pct': 100.0 * (n - nex) / max(t, 1),
                             'area': [lo, hi, p0, p1]})

    # 障碍物：最近占用格有多远（进图了就该 ≈0）+ 边界覆盖率
    for name, x0, x1, y0, y1 in obstacles:
        dmin = float(dist_to_walls(X[occ], Y[occ], [(name, x0, x1, y0, y1)]).min()) \
            if occ.any() else float('inf')
        oh, ot = coverage_of_segments(m, [(x0, x1, y0, y1)], 0.10)
        out['obstacles'].append({
            'name': name, 'rect': [x0, x1, y0, y1],
            'min_dist_cm': 100 * dmin, 'perimeter_hit': oh, 'perimeter_cells': ot,
            'perimeter_pct': 100.0 * oh / max(ot, 1)})

    cname, cseg = longest_wall_segment(walls)
    chit, ctot = coverage_of_segments(m, [cseg])
    out['control'] = {'wall': cname, 'segment': list(cseg), 'hit': chit,
                      'cells': ctot, 'pct': 100.0 * chit / max(ctot, 1)}

    if do_shift:
        mm, dx, dy = best_shift(X, Y, occ, solids)
        out['shift'] = {'dx_cm': 100 * dx, 'dy_cm': 100 * dy, 'mean_cm': 100 * mm}
    return out


def print_report(o):
    print(f"\n{'=' * 72}\n{o['map']}\n  world: {o['world']}\n{'=' * 72}")
    print(f"尺寸        : {o['width']} x {o['height']} 格 @ {o['resolution']} m"
          f"  =  {o['size_m'][0]:.2f} x {o['size_m'][1]:.2f} m")
    print(f"原点        : ({o['origin'][0]:.3f}, {o['origin'][1]:.3f})")
    tot = o['width'] * o['height']
    print(f"占用/空闲/未知: {o['occupied']} ({o['occupied_pct']:.1f}%) / "
          f"{o['free']} ({100*o['free']/tot:.1f}%) / {o['unknown']} ({100*o['unknown']/tot:.1f}%)")
    d = o['dist_cm']
    print(f"\n-- 占用格到最近真实墙面 --")
    print(f"  均值 {d['mean']:.2f} cm | 中位 {d['median']:.2f} cm | 最大 {d['max']:.2f} cm")
    print(f"  ≤  5 cm : {d['le5']:6.2f}%   ≤ 10 cm : {d['le10']:6.2f}%   "
          f"≤ 15 cm : {d['le15']:6.2f}%")
    print(f"\n-- 幻影墙（整块离真实墙 >15 cm）--")
    print(f"  连通分量 {o['components']} 个；离墙 >15cm 的 {o['phantom_blocks']} 个"
          f"（≥4 格的 {o['phantom_big']} 个）")
    print(f"\n-- 可见墙面覆盖率（墙面采样点 10 cm 内有占用格）--")
    print(f"  {o['coverage'][0]}/{o['coverage'][1]} = {o['coverage_pct']:.2f}%")
    print(f"\n-- 门洞通畅度（门洞里的占用格，理想 0；门里本来就有障碍物的会剔除）--")
    for dr in o['doors']:
        flag = '✅' if dr['pct'] < 25 else ('⚠️' if dr['pct'] < 50 else '❌')
        extra = (f"（其中 {dr['occupied_in_obstacle']} 格是门里那个障碍物本身）"
                 if dr['occupied_in_obstacle'] else '')
        print(f"  {flag} {dr['label']:<26s} {dr['occupied']:>3d}/{dr['cells']:<3d}"
              f" 占用，净 {dr['occupied_net']:>3d} ({dr['pct']:5.1f}%){extra}")
    c = o['control']
    print(f"  对照 · 最长墙({c['wall']})中段 1 m 的覆盖率 : "
          f"{c['hit']}/{c['cells']} = {c['pct']:.1f}%")
    if o['obstacles']:
        print(f"\n-- 障碍物进图情况（最近的占用格离它多远 / 边界被覆盖多少）--")
        for ob in o['obstacles']:
            flag = '✅' if ob['min_dist_cm'] <= 5.0 else (
                '⚠️' if ob['min_dist_cm'] <= 15.0 else '❌')
            print(f"  {flag} {ob['name']:<18s} 最近占用格 {ob['min_dist_cm']:5.1f} cm"
                  f" | 边界覆盖 {ob['perimeter_hit']:>3d}/{ob['perimeter_cells']:<3d}"
                  f" = {ob['perimeter_pct']:5.1f}%")
        print("  （边界覆盖率不会到 100%：背对机器人轨迹的那几面雷达本来就看不到）")
    if 'shift' in o:
        s = o['shift']
        print(f"\n-- 最佳整体平移（对齐 world）--")
        print(f"  dx {s['dx_cm']:+.1f} cm, dy {s['dy_cm']:+.1f} cm → 平均距离 {s['mean_cm']:.2f} cm")


def render_png(map_yaml, world_sdf, out_png, scale=4):
    """画一张"地图 vs 真实几何"的对照图。

    黑色 = 占用格，白色 = 空闲，灰色 = 未知；
    蓝框 = world 里的结构墙，红框 = obs_* 障碍物的**未旋转 AABB**。
    斜着放的障碍物（obs_roomB_slant）框会比实体大一圈，这是 AABB 的固有
    保守性，不是地图错了。
    """
    from PIL import Image, ImageDraw                    # noqa: PLC0415

    m = load_map(map_yaml)
    solids = parse_world(world_sdf)
    h, w, res, ox, oy = m['h'], m['w'], m['res'], m['ox'], m['oy']
    img = Image.new('RGB', (w, h), (128, 128, 128))
    px = img.load()
    for r in range(h):
        for c in range(w):
            if m['occ'][r, c]:
                px[c, r] = (0, 0, 0)
            elif m['free'][r, c]:
                px[c, r] = (255, 255, 255)
    img = img.resize((w * scale, h * scale), Image.NEAREST)
    dr = ImageDraw.Draw(img)
    try:
        from PIL import ImageFont                       # noqa: PLC0415

        def load(path, size=13):
            return ImageFont.truetype(path, size) if os.path.isfile(path) else None

        # 默认位图字体在 800px 宽的图上几乎看不清。
        # 而且没有哪个字体两边都行：DejaVu 没有中文字形（中文变豆腐块），
        # DroidSansFallback 反过来把 ASCII 也画成豆腐块。所以中文标题和
        # 英文实体名各用一个字体（踩过两次）。
        font_cjk = load('/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')
        font_latin = load('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    except Exception:                                   # noqa: BLE001
        font_cjk = font_latin = None

    def to_px(x, y):
        return ((x - ox) / res * scale, (oy + h * res - y) / res * scale)

    for name, x0, x1, y0, y1 in solids:
        col = (220, 40, 40) if name.startswith('obs_') else (40, 80, 220)
        a, b = to_px(x0, y1), to_px(x1, y0)
        dr.rectangle([a, b], outline=col, width=2)
        dr.text((a[0] + 3, a[1] + 3), name, fill=col, font=font_latin)
    dr.text((8, 6), f'{os.path.basename(map_yaml)}   {w}x{h} @ {res} m   '
                    f'黑=占用格 灰=未知 蓝框=真实墙 红框=真实障碍物',
            fill=(0, 130, 0), font=font_cjk)
    img.save(out_png)
    return out_png


def default_world():
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('my_robot_description'),
                         'worlds', 'rooms.sdf')
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    p = os.path.join(here, '..', 'worlds', 'rooms.sdf')
    return p if os.path.isfile(p) else None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map', help='地图 yaml（pgm 用 yaml 里的 image 字段）')
    ap.add_argument('--world', default=None, help='world SDF，默认包内 worlds/rooms.sdf')
    ap.add_argument('--json', action='store_true', help='输出 JSON')
    ap.add_argument('--no-shift', action='store_true', help='跳过整体平移搜索（更快）')
    ap.add_argument('--png', default=None,
                    help='额外输出一张"地图 vs 真实几何"的对照图（黑=占用，蓝=墙，红=障碍物）')
    a = ap.parse_args(argv)

    world = a.world or default_world()
    if not world or not os.path.isfile(world):
        raise SystemExit(f'找不到 world 文件: {world}')
    o = evaluate(a.map, world, do_shift=not a.no_shift)
    if a.json:
        print(json.dumps(o, ensure_ascii=False, indent=2))
    else:
        print_report(o)
    if a.png:
        render_png(a.map, world, a.png)
        print(f'\n对照图已写出: {a.png}')
    # 门洞被堵死、或障碍物压根没进图，都算失败，方便脚本化
    bad_door = any(d['pct'] >= 50 for d in o['doors'])
    missed = [ob['name'] for ob in o['obstacles'] if ob['min_dist_cm'] > 5.0]
    if missed:
        print(f'\n❌ 这些障碍物没有进图: {", ".join(missed)}', file=sys.stderr)
    return 1 if (bad_door or missed) else 0


if __name__ == '__main__':
    sys.exit(main())
