#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从仿真 Nav2 参数生成**真车** Nav2 参数 —— 单一数据源，不手工维护第二份。

问题背景
========
`config/nav2_params.yaml` 是从 Jazzy 上游派生的**仿真**参数：它的 footprint 和
速度上限来自 `urdf/my_robot.urdf.xacro`（仿真车 0.40 x 0.37 m、0.5 m/s）。
而真机 bringup 早先没有指定 params_file，于是**真车也在用这份仿真参数**。

后果有两条，都不会报错，只是表现不正常：

  1. 车被当成两倍大。真车外接圆 0.146 m，仿真 0.272 m。Nav2 认为车过不去
     的门和缝，它会**拒绝走**。
  2. 速度打架。Nav2 按 0.5 m/s 规划，驱动按 max_linear_vel=0.20 截断。
     MPPI 会**永远认为自己欠跟踪**，表现为反复重规划、走走停停。

设计：为什么是"生成"而不是"拷一份"
==================================
本工程约定"同一个数值只登记一处"。所以这里不手写第二份参数，而是：

    urdf/acg720_real.urdf.xacro  ─┐
                                  ├─> scripts/gen_real_params.py ─> config/nav2_params_real.yaml
    launch/robot.launch.py       ─┘

真车的几何从 **URDF 的 xacro 属性**读，速度上限从 **robot.launch.py 的
DeclareLaunchArgument 默认值**读。两处都是运行时的真实来源，不存在冗余登记。

改动方式是**逐行文本替换**，不用 yaml.safe_load/dump —— 后者会丢掉源文件里
全部的 `★` 注释，而那份注释是参数推导过程的唯一记录。

一致性由 test/test_real_params.py 守住：改了源文件却忘了重新生成，测试会红。

用法
====
    ros2 run my_robot_description gen_real_params.py            # 重新生成
    ros2 run my_robot_description gen_real_params.py --check    # 只检查是否最新
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys

# 膨胀半径规则：外接圆半径 + 余量。
# 来源 docs/real_robot_params.md：「它应该约等于"外接圆半径 + 5~8 cm 余量"」。
# 仿真那份用的也是同一条规则：外接圆 0.272 + 0.078 = 0.35 ✓
INFLATION_MARGIN_M = 0.06

GENERATED_MARK = '本文件由 scripts/gen_real_params.py 自动生成'


# --------------------------------------------------------------- 源：真车几何
def read_xacro_properties(xacro_path: str) -> dict[str, float]:
    """从 xacro 里读纯数字的 <xacro:property>。

    只收 value 是字面数字的项；带 ${} 表达式的（如 wheel_offset_x =
    ${wheelbase / 2}）由 derive_geometry() 自己算，不在这里求值 ——
    在 Python 里实现一个 xacro 表达式求值器不值得。
    """
    with open(xacro_path, 'r', encoding='utf-8') as f:
        text = f.read()
    props: dict[str, float] = {}
    for m in re.finditer(
            r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"', text):
        name, raw = m.group(1), m.group(2)
        if '${' in raw:
            continue
        try:
            props[name] = float(raw)
        except ValueError:
            continue
    return props


def derive_geometry(xacro_path: str) -> dict[str, float]:
    """由 URDF 属性推导 footprint 半长/半宽与外接圆半径。

    footprint 取**底盘和轮子的并集**：轮子比底盘宽的时候不能只按底盘算，
    否则会漏碰（仿真那份参数里专门写了这一条）。
    """
    p = read_xacro_properties(xacro_path)
    need = ('wheelbase', 'track_width', 'wheel_width',
            'base_length', 'base_width')
    missing = [k for k in need if k not in p]
    if missing:
        raise SystemExit(f'❌ {xacro_path} 里缺这些 property: {missing}')

    wheel_offset_x = p['wheelbase'] / 2.0
    wheel_offset_y = p['track_width'] / 2.0
    half_x = max(p['base_length'] / 2.0, wheel_offset_x)
    half_y = max(p['base_width'] / 2.0, wheel_offset_y + p['wheel_width'] / 2.0)
    return {
        'half_x': half_x,
        'half_y': half_y,
        'circumradius': math.hypot(half_x, half_y),
        'base_length': p['base_length'],
        'base_width': p['base_width'],
        'wheelbase': p['wheelbase'],
        'track_width': p['track_width'],
        'wheel_width': p['wheel_width'],
    }


# ------------------------------------------------------------- 源：驱动限幅
def read_launch_default(launch_path: str, arg: str) -> float:
    """读 robot.launch.py 里某个 DeclareLaunchArgument 的默认值。

    驱动的限幅和 Nav2 的速度上限必须一致，否则又回到"规划 0.5 实际 0.2"。
    所以这里不写死数字，而是去 launch 里读 —— 那才是运行时的真实来源。
    """
    with open(launch_path, 'r', encoding='utf-8') as f:
        text = f.read()
    m = re.search(rf"'{re.escape(arg)}',\s*default_value='([-0-9.]+)'", text)
    if not m:
        raise SystemExit(f'❌ 在 {launch_path} 里找不到 {arg} 的 default_value')
    return float(m.group(1))


def _num(v: float) -> str:
    """把浮点格式化成**一定带小数点**的字符串。

    ★ 这里不能图省事直接用 f'{v:g}'：1.0 会被写成 `1`，而 ROS 2 是从 YAML 的
      字面量推断参数类型的 —— `wz_max: 1` 会被当成整数，节点声明的却是 double，
      结果启动时报参数类型不匹配。数值上"看着完全对"，节点却起不来。
    """
    s = f'{v:g}'
    if '.' in s or 'e' in s or 'inf' in s or 'nan' in s:
        return s
    return s + '.0'


# ------------------------------------------------------------------ 改动表
def build_overrides(pkg_dir: str) -> list[tuple[str, str, str]]:
    """构造 (参数名, 新值, 理由) 列表。顺序即报告里的顺序。"""
    geo = derive_geometry(os.path.join(
        pkg_dir, 'urdf', 'acg720_real.urdf.xacro'))
    launch = os.path.join(pkg_dir, 'launch', 'robot.launch.py')
    v_max = read_launch_default(launch, 'max_linear_vel')
    w_max = read_launch_default(launch, 'max_angular_vel')

    hx, hy = geo['half_x'], geo['half_y']
    circum = geo['circumradius']
    infl = round(circum + INFLATION_MARGIN_M, 2)

    fp = (f'"[[{_num(-hx)}, {_num(-hy)}], [{_num(hx)}, {_num(-hy)}], '
          f'[{_num(hx)}, {_num(hy)}], [{_num(-hx)}, {_num(hy)}]]"')
    geom_src = ('从 urdf/acg720_real.urdf.xacro 推导：'
                f'半长 {hx:g}（轴距/2）、半宽 {hy:g}（轮距/2 + 胎宽/2）')
    infl_src = (f'外接圆 {circum:.4f} + 余量 {INFLATION_MARGIN_M} '
                f'（规则见 docs/real_robot_params.md）')
    vel_src = f'对齐 launch/robot.launch.py 的默认限幅 {v_max:g}/{w_max:g}'

    return [
        ('footprint', fp, geom_src),
        ('inflation_radius', _num(infl), infl_src),
        ('vx_max', _num(v_max), vel_src),
        ('vx_min', _num(-v_max), vel_src + '（取对称）'),
        ('vy_max', '0.0', '差速车没有横向运动，仿真里的 0.5 是无意义的'),
        ('wz_max', _num(w_max), vel_src),
        ('max_velocity', f'[{_num(v_max)}, 0.0, {_num(w_max)}]', vel_src),
        ('min_velocity', f'[{_num(-v_max)}, 0.0, {_num(-w_max)}]', vel_src),
    ]


# --------------------------------------------------------------- 文本替换
def _values_of(text: str, key: str) -> list[str]:
    return re.findall(rf'^\s*{re.escape(key)}\s*:\s*(.*?)\s*$', text,
                      re.MULTILINE)


def _sub_key(text: str, key: str, new_value: str) -> tuple[str, int]:
    """把 `   key: 任意` 这一行替换成 `   key: new_value`，保留缩进。

    正则要求 key 后面紧跟可选的空白再是冒号，所以 `footprint` 不会误伤
    `footprint_topic`。同一个 key 出现多次（局部/全局代价地图各一份
    footprint、各一份 inflation_radius）会**全部**替换，这正是想要的。
    """
    pat = re.compile(rf'^(\s*){re.escape(key)}\s*:\s*.*$', re.MULTILINE)
    out, n = pat.subn(lambda m: f'{m.group(1)}{key}: {new_value}', text)
    return out, n


def generate(src_text: str, overrides: list[tuple[str, str, str]]) -> str:
    rows: list[tuple[str, str, int, str, str]] = []
    out = src_text
    for key, new, reason in overrides:
        olds = _values_of(out, key)
        if not olds:
            raise SystemExit(
                f'❌ 源文件里找不到参数 `{key}`，生成器中止。'
                f'（上游改了 key 名？需要同步 scripts/gen_real_params.py）')
        uniq = sorted(set(olds))
        out, n = _sub_key(out, key, new)
        if n != len(olds):
            raise SystemExit(f'❌ `{key}` 替换数 {n} != 命中数 {len(olds)}')
        rows.append((key, uniq[0] if len(uniq) == 1 else ' | '.join(uniq),
                     n, new, reason))

    return _header(rows) + out


def _header(rows: list[tuple[str, str, int, str, str]]) -> str:
    line = '=' * 76
    L = [
        f'# {line}',
        f'# ★★★ {GENERATED_MARK}，**不要手工编辑** ★★★',
        f'# {line}',
        '# 源: config/nav2_params.yaml + urdf/acg720_real.urdf.xacro'
        ' + launch/robot.launch.py',
        '# 重新生成: ros2 run my_robot_description gen_real_params.py',
        '# 一致性:   test/test_real_params.py（改了源文件忘了重新生成 -> 测试红）',
        '#',
        '# 为什么真车不能直接用仿真那份',
        '#   仿真车和真车是同一台车的两个模型，几何和限幅都不同：',
        '#                   仿真                  真车',
        '#     footprint     0.40 x 0.37 m         见下表',
        '#     外接圆        0.272 m               见下表',
        '#     vx_max        0.5 m/s               见下表（驱动限幅）',
        '#     wz_max        1.5 rad/s             见下表（驱动限幅）',
        '#   误用的后果：车被当成两倍大（能过的门不走），且 Nav2 按 0.5 规划、',
        '#   驱动按 0.2 截断，MPPI 持续欠跟踪 -> 走走停停。两者都不报错。',
        '#',
        '# 相对源文件的**全部**改动：',
        '#   参数                源文件里的值                 ->  本文件',
    ]
    for key, old, n, new, reason in rows:
        cnt = f' (x{n})' if n > 1 else ''
        L.append(f'#   {key + cnt:<20} {old:<28} ->  {new}')
        L.append(f'#       └─ {reason}')
    L += [
        '#',
        '# 未改动的项（沿用仿真值，【未标定】，上车后按需再调）:',
        '#   max_accel / max_decel  —— 真车没有实测依据，不凭空编造数字。',
        '#       0.5 m/s^2 / 2.0 rad/s^2 相对 0.2 m/s 的上限不算激进。',
        '#   amcl.laser_min_range / max_range —— 取决于雷达型号，【待确认】。',
        '#   raytrace_max_range 等传感器参数 —— 同上。',
        f'# {line}',
        '',
    ]
    return '\n'.join(L)


# ---------------------------------------------------------------------- main
def _paths() -> tuple[str, str, str]:
    """定位源文件与目标文件所在的包目录。

    优先**源码树**：本工程按 `colcon build --symlink-install` 使用，install 里
    是符号链接，所以 realpath 能回到 src/ 下的真实目录 —— 生成结果直接写回
    源码树，才是应该提交进 git 的那一份。

    装成非符号链接时回退到 ament 的 share 目录（那里有 config/ urdf/ launch/，
    没有 scripts/，但生成只需要前三者）。
    """
    here = os.path.dirname(os.path.realpath(__file__))
    cands = [os.path.dirname(here)]          # 源码树: <pkg>/scripts -> <pkg>
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(get_package_share_directory('my_robot_description'))
    except Exception:                        # noqa: BLE001 - ROS 环境可能没 source
        pass
    for pkg_dir in cands:
        src = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
        if os.path.isfile(src):
            return (src,
                    os.path.join(pkg_dir, 'config', 'nav2_params_real.yaml'),
                    pkg_dir)
    raise SystemExit('❌ 找不到 config/nav2_params.yaml，试过:\n  ' +
                     '\n  '.join(cands))


def main() -> int:
    ap = argparse.ArgumentParser(
        description='从仿真 Nav2 参数生成真车 Nav2 参数（单一数据源）')
    ap.add_argument('--check', action='store_true',
                    help='只检查磁盘上的文件是否是最新的；过期则退出码 1')
    args = ap.parse_args()

    src, dst, pkg_dir = _paths()
    if not os.path.isfile(src):
        raise SystemExit(f'❌ 找不到源参数文件: {src}')

    with open(src, 'r', encoding='utf-8') as f:
        src_text = f.read()
    out = generate(src_text, build_overrides(pkg_dir))

    old = None
    if os.path.isfile(dst):
        with open(dst, 'r', encoding='utf-8') as f:
            old = f.read()

    if args.check:
        if old == out:
            print(f'✓ {os.path.basename(dst)} 是最新的')
            return 0
        print(f'✗ {os.path.basename(dst)} 与源文件不一致，请重新生成：')
        print('    ros2 run my_robot_description gen_real_params.py')
        return 1

    if old == out:
        print(f'= {os.path.basename(dst)} 无变化')
        return 0
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(out)
    print(f'✓ 已生成 {dst}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
