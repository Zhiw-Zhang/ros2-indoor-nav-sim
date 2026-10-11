#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真车 Nav2 参数的一致性测试。

守住的核心不变量
================
`config/nav2_params_real.yaml` **是生成物**，源有两个：

    urdf/acg720_real.urdf.xacro   （几何）
    launch/robot.launch.py        （驱动限幅）

所以这里要防三类漂移：

  1. 改了源文件却忘了重新生成 -> 磁盘上的参数过期（test_generated_file_is_current）
  2. 生成器把浮点写成了整数 -> ROS 2 参数类型不匹配、节点起不来
     （test_float_values_keep_decimal_point）
  3. 有人手工编辑了生成物 -> 下次生成就被覆盖，且与源不符
     （同样由 test_generated_file_is_current 兜住）

另外还独立复核了推导本身的正确性（footprint/膨胀半径/限幅），
这样即使生成器算错了，测试也会发现。
"""

from __future__ import annotations

import math
import pathlib
import re
import sys

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / 'scripts'))

import gen_real_params as gen  # noqa: E402

SIM_YAML = PKG / 'config' / 'nav2_params.yaml'
REAL_YAML = PKG / 'config' / 'nav2_params_real.yaml'
URDF = PKG / 'urdf' / 'acg720_real.urdf.xacro'
LAUNCH = PKG / 'launch' / 'robot.launch.py'


def _text(p: pathlib.Path) -> str:
    return p.read_text(encoding='utf-8')


def _values(text: str, key: str) -> list[str]:
    return re.findall(rf'^\s*{re.escape(key)}\s*:\s*(.*?)\s*$', text,
                      re.MULTILINE)


def _one(text: str, key: str) -> str:
    vs = set(_values(text, key))
    assert len(vs) == 1, f'{key} 的值不唯一或缺失: {vs}'
    return vs.pop()


# ------------------------------------------------------- 1. 生成物是否最新
def test_generated_file_is_current():
    """磁盘上的真车参数 == 现在重新生成的结果。"""
    expected = gen.generate(_text(SIM_YAML),
                            gen.build_overrides(str(PKG)))
    assert REAL_YAML.is_file(), '真车参数文件不存在，跑 gen_real_params.py 生成'
    assert _text(REAL_YAML) == expected, (
        'config/nav2_params_real.yaml 与源文件不一致（手工改过？忘了重新生成？）。\n'
        '  重新生成: ros2 run my_robot_description gen_real_params.py')


def _circumradius(fp_value: str) -> float:
    """从 footprint 字符串算外接圆半径。"""
    pts = yaml.safe_load(fp_value.strip('"'))
    hx = max(abs(p[0]) for p in pts)
    hy = max(abs(p[1]) for p in pts)
    return math.hypot(hx, hy)


# ------------------------------------------------------ 2. 文件本身能解析
def test_generated_yaml_parses():
    data = yaml.safe_load(_text(REAL_YAML))
    assert isinstance(data, dict)
    # 关键的几节必须在
    for section in ('amcl', 'controller_server', 'local_costmap',
                    'global_costmap', 'velocity_smoother', 'collision_monitor'):
        assert section in data, f'缺 {section} 节'


# ------------------------------------------------ 3. 几何与源 URDF 一致
def test_footprint_matches_real_urdf():
    geo = gen.derive_geometry(str(URDF))
    fp = _one(_text(REAL_YAML), 'footprint').strip('"')
    pts = yaml.safe_load(fp)
    assert len(pts) == 4
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    assert max(xs) == pytest.approx(geo['half_x'], abs=1e-9)
    assert max(ys) == pytest.approx(geo['half_y'], abs=1e-9)
    # 轮子比底盘宽时不能只按底盘算，否则漏碰
    assert geo['half_y'] > geo['base_width'] / 2, '半宽应当被轮子撑大'


def test_footprint_added_margin_is_symmetric():
    pts = yaml.safe_load(_one(_text(REAL_YAML), 'footprint').strip('"'))
    xs = sorted(p[0] for p in pts)
    ys = sorted(p[1] for p in pts)
    assert xs[0] == pytest.approx(-xs[-1])
    assert ys[0] == pytest.approx(-ys[-1])


def test_no_sim_geometry_left():
    """仿真那套大几何不该有任何**参数行**残留（两份 footprint 都要换掉）。

    ★ 只看非注释行：生成文件的头部**故意**把仿真的旧值列在改动对照表里，
      那是参数推导过程的记录，不该被当成"没改干净"。
    """
    body = '\n'.join(line for line in _text(REAL_YAML).splitlines()
                     if not line.lstrip().startswith('#'))
    sim_fp = _one(_text(SIM_YAML), 'footprint')
    assert sim_fp not in body, '真车参数的 footprint 行里还留着仿真的值'
    assert _values(body, 'footprint').count(sim_fp) == 0


def test_inflation_radius_follows_rule():
    """膨胀半径 = 外接圆 + 余量（规则来自 docs/real_robot_params.md）。"""
    geo = gen.derive_geometry(str(URDF))
    infl = float(_one(_text(REAL_YAML), 'inflation_radius'))
    assert infl == pytest.approx(
        round(geo['circumradius'] + gen.INFLATION_MARGIN_M, 2))
    # 至少不能小于外接圆，否则本该膨胀掉的障碍会贴到车上
    assert infl > geo['circumradius']


def test_both_costmaps_share_footprint_and_inflation():
    """局部和全局代价地图必须一致，否则规划与实际判定打架。"""
    real = _text(REAL_YAML)
    assert len(set(_values(real, 'footprint'))) == 1
    assert len(set(_values(real, 'inflation_radius'))) == 1
    assert len(_values(real, 'footprint')) == 2, '应当有两处 footprint'


# --------------------------------------------- 4. 速度上限与驱动限幅一致
def test_velocity_limits_match_driver_defaults():
    """Nav2 的速度上限必须 == 驱动的限幅，否则 MPPI 会持续欠跟踪。"""
    v_max = gen.read_launch_default(str(LAUNCH), 'max_linear_vel')
    w_max = gen.read_launch_default(str(LAUNCH), 'max_angular_vel')
    real = _text(REAL_YAML)

    assert float(_one(real, 'vx_max')) == pytest.approx(v_max)
    assert float(_one(real, 'wz_max')) == pytest.approx(w_max)
    assert yaml.safe_load(_one(real, 'max_velocity')) == pytest.approx(
        [v_max, 0.0, w_max])
    assert yaml.safe_load(_one(real, 'min_velocity')) == pytest.approx(
        [-v_max, 0.0, -w_max])


def test_velocity_smoother_respects_mppi_limits():
    """smoother 的上限不能比 MPPI 大，否则两边互相打架。"""
    real = _text(REAL_YAML)
    vx_max = float(_one(real, 'vx_max'))
    wz_max = float(_one(real, 'wz_max'))
    mv = yaml.safe_load(_one(real, 'max_velocity'))
    assert mv[0] <= vx_max + 1e-9
    assert mv[2] <= wz_max + 1e-9


def test_differential_drive_has_no_lateral_motion():
    """差速/skid-steer 没有横向运动，vy 上限必须是 0。"""
    assert float(_one(_text(REAL_YAML), 'vy_max')) == 0.0
    mv = yaml.safe_load(_one(_text(REAL_YAML), 'max_velocity'))
    assert mv[1] == 0.0


def test_nav2_is_slower_than_or_equal_to_sim():
    """真车参数必须**不比仿真激进**，这是回归红线。"""
    sim, real = _text(SIM_YAML), _text(REAL_YAML)
    assert float(_one(real, 'vx_max')) <= float(_one(sim, 'vx_max'))
    assert float(_one(real, 'wz_max')) <= float(_one(sim, 'wz_max'))
    assert float(_one(real, 'inflation_radius')) <= float(
        _one(sim, 'inflation_radius'))
    assert _circumradius(_one(real, 'footprint')) < _circumradius(
        _one(sim, 'footprint'))


# ------------------------------------------- 5. 浮点必须带小数点（类型安全）
def test_float_values_keep_decimal_point():
    """★ 回归 bug：f'{1.0:g}' == '1'，ROS 2 会把它当整数 -> 参数类型不匹配。

    节点起不来，但 YAML 看起来完全正常。
    """
    real = _text(REAL_YAML)
    float_keys = ('inflation_radius', 'vx_max', 'vx_min', 'vy_max', 'wz_max',
                  'vx_std', 'wz_std')
    for key in float_keys:
        for v in _values(real, key):
            v = v.strip().strip('"')
            assert '.' in v, (
                f'{key}: {v} 看起来是整数。ROS 2 按 YAML 字面量推断类型，'
                f'节点声明的是 double，会报参数类型不匹配。')
    for key in ('max_velocity', 'min_velocity'):
        for v in _values(real, key):
            for num in yaml.safe_load(v):
                if isinstance(num, int):
                    pytest.fail(f'{key} 里有整数 {num}，应为浮点 {num}.0')


def test_launch_uses_real_params_by_default():
    """robot.launch.py 必须默认指向真车参数，且真的把它传下去。"""
    launch = _text(LAUNCH)
    assert 'nav2_params_real.yaml' in launch
    assert "'params_file': params_file" in launch, '参数没传给 nav.launch.py'
