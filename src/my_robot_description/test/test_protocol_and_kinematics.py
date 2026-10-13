#!/usr/bin/env python3
"""四轮运动学与位姿积分的单元测试。

⚠ 本文件原本还含"旧协议"（凭空发明的那套 MODBUS 帧）的测试。
   那套协议**跟真固件说不上一句话**（第 3 个字节就会被判版本错），
   已经整体作废：真协议在 `acg720_protocol_v1.py`，它的 68 条测试在
   `test_protocol_v1.py`（含与厂商实现的活体交叉核对）。
   这里只留下**与协议无关**的部分：轮位 ↔ 左右、几何换算、位姿积分。

为什么这些值得单独测：符号约定和尺度是"接上真车才发现"的两类 bug，
而真车调试的代价远高于这里写测试的代价 —— 左右轮搞反了车会反向转弯，
日志上却看起来一切正常。

跑法：
    pytest src/my_robot_description/test/test_protocol_and_kinematics.py -q
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from drive_kinematics import (  # noqa: E402
    Pose2D, SkidSteerGeometry, lr_to_wheels, move_to_twist,
    quaternion_from_yaw, twist_to_move, wheels_to_lr, wrap_angle,
)

# ==========================================================================
# 四轮 ↔ 左右 的归一化
# ==========================================================================

def test_wheels_to_lr_matches_handoff_normalization():
    """交接包 4.4 节：前进=(A+B+C+D)/4，左旋=(A−B−C+D)/4。

    用纯前进 (v,0) 和纯左旋 (0,ω) 两个工况反解，验证本文件的左右定义
    与 FPGA 的归一化**数学等价**。
    """
    # 纯前进：四轮同速 → 左右相等
    left, right = wheels_to_lr(20, 20, 20, 20)
    assert left == right == 20
    # 纯左旋：B=C=+20（左轮正转），A=D=−20（右轮反转）
    left, right = wheels_to_lr(-20, 20, 20, -20)
    assert left == 20 and right == -20


def test_lr_to_wheels_is_inverse():
    for left, right in [(0, 0), (20, 20), (10, 40), (-5, 7.5)]:
        a, b, c, d = lr_to_wheels(left, right)
        assert wheels_to_lr(a, b, c, d) == pytest.approx((left, right))


def test_f1_acceptance_case_reproduces_handoff_numbers():
    """用交接包 F1 验收里真实出现过的一组目标值回归。

    右弯 40RPM 轮速比 192/256 → 目标 (A,B,C,D) = (30,40,40,30)。
    左右应变 30 / 40，且归一化前进/左旋要能对上。
    """
    a, b, c, d = 30, 40, 40, 30
    left, right = wheels_to_lr(a, b, c, d)
    assert (left, right) == (40.0, 30.0)
    forward = (a + b + c + d) / 4.0          # FIFO 归一化
    left_spin = (a - b - c + d) / 4.0
    assert forward == 35.0
    assert left_spin == -5.0                  # 负 = 右旋
    assert (left + right) / 2 == forward
    assert (left - right) / 2 == -left_spin   # 左旋为正，与 FPGA 符号相反


# ==========================================================================
# 运动学
# ==========================================================================

def test_geometry_defaults_match_handoff():
    g = SkidSteerGeometry()
    assert g.wheel_radius_m == 0.0325          # 65 mm 轮径
    assert g.wheel_separation_m == 0.200       # 200 mm 轮距
    assert g.wheelbase_m == 0.185              # 185 mm 轴距
    assert g.counts_per_rev == 1320.0
    assert g.scale == 1.0


def test_counts_per_meter():
    g = SkidSteerGeometry()
    circ = 2 * math.pi * 0.0325
    assert g.wheel_circumference_m == pytest.approx(circ)
    assert g.counts_per_meter == pytest.approx(1320.0 / circ)
    # 1320 counts / (2π·0.0325) ≈ 6464 counts/m
    assert g.counts_per_meter == pytest.approx(6464.0, rel=1e-3)


def test_rpm_per_mps():
    g = SkidSteerGeometry()
    # 60/(2π·0.0325) ≈ 293.9 RPM per m/s
    assert g.rpm_per_mps == pytest.approx(293.9, rel=1e-3)
    assert g.rpm_to_mps(g.mps_to_rpm(0.37)) == pytest.approx(0.37)


def test_twist_wheels_roundtrip_uses_effective_separation():
    """Twist → 轮速 → Twist 必须闭环，且 scale 生效。"""
    for scale in (0.8, 1.0, 1.36, 2.0):
        g = SkidSteerGeometry(scale=scale)
        left, right = g.twist_to_wheels(0.3, 0.5)
        v, omega = g.wheels_to_twist(left, right)
        assert v == pytest.approx(0.3)
        assert omega == pytest.approx(0.5)


def test_scale_1_36_needs_more_wheel_differential():
    """scale > 1 意味着"要转同样的角，轮子必须差得更多"，用来补偿侧滑。"""
    base = SkidSteerGeometry(scale=1.0)
    corr = SkidSteerGeometry(scale=1.36)
    l0, r0 = base.twist_to_wheels(0.0, 1.0)
    l1, r1 = corr.twist_to_wheels(0.0, 1.0)
    assert (r1 - l1) > (r0 - l0)
    assert (r1 - l1) == pytest.approx((r0 - l0) * 1.36)


def test_pure_forward_gives_equal_wheels():
    g = SkidSteerGeometry()
    left, right = g.twist_to_wheels(0.2, 0.0)
    assert left == pytest.approx(right)
    assert left == pytest.approx(0.2 * g.rpm_per_mps)


def test_pure_rotation_sign_convention():
    """左旋 ω>0 → 右轮快于左轮（ROS REP-103：z 轴向上，逆时针为正）。"""
    g = SkidSteerGeometry()
    left, right = g.twist_to_wheels(0.0, 1.0)
    assert right > left
    # 左旋时左轮应该是反转
    assert left < 0 < right


def test_counts_to_distance_and_rotation():
    g = SkidSteerGeometry()
    d = g.counts_to_distance_m(1320)
    assert d == pytest.approx(g.wheel_circumference_m)
    assert g.distance_to_counts(d) == pytest.approx(1320)

    # 右轮走一圈、左轮走一圈的负值 → 原地转，转角 = 2·(πD) / b_eff
    rot = g.counts_to_rotation_rad(-1320, 1320)
    expected = 2 * g.wheel_circumference_m / g.effective_separation_m
    assert rot == pytest.approx(expected)


def test_effective_separation_scales_rotation():
    """同样轮差，scale 越大报出的转角越小 —— 这就是里程计尺度的作用。"""
    raw = SkidSteerGeometry(scale=1.0).counts_to_rotation_rad(-1320, 1320)
    cal = SkidSteerGeometry(scale=1.36).counts_to_rotation_rad(-1320, 1320)
    assert cal == pytest.approx(raw / 1.36)


# ==========================================================================
# 位姿积分
# ==========================================================================

def test_pose_straight_line():
    p = Pose2D()
    for _ in range(20):            # 20 × 50 ms = 1 s
        p = p.integrate(1.0, 0.0, 0.05)
    assert p.x == pytest.approx(1.0)
    assert p.y == pytest.approx(0.0)
    assert p.yaw == pytest.approx(0.0)


def test_pose_full_circle_closes():
    """原地转 2π 必须回到原点、yaw 回 0 —— 中点积分应当几乎无漂移。"""
    omega = 2 * math.pi / 2.0      # 2 秒转一圈
    p = Pose2D()
    for _ in range(40):            # 2 s / 50 ms
        p = p.integrate(0.0, omega, 0.05)
    assert p.x == pytest.approx(0.0, abs=1e-12)
    assert p.y == pytest.approx(0.0, abs=1e-12)
    assert p.yaw == pytest.approx(0.0, abs=1e-9)


def test_pose_arc_start_and_end():
    """走一段圆弧：起点朝 +x，半径 R=v/ω，转约 π/2 后位置应到约 (R, R)。

    ⚠ 注意这里**不**断言 yaw 精确等于 π/2。采样周期 dt=0.05 s、ω=0.5 rad/s
    时，π/2 需要 15.71 个采样——不是整数。离散化误差是 **O(ω·dt) ≈ 0.021 rad**，
    这是格式本身的属性，不是 bug，所以这里分别钉住两件事：
      1. 转角精确等于 n·ω·dt（离散积分的解析真值）；
      2. 离散化误差确实被 ω·dt 这个量级界住（证明没有实现错误）。
    """
    v, omega, dt = 0.3, 0.5, 0.05
    R = v / omega

    n = int(round((math.pi / 2 / omega) / dt))      # 16 个采样 ≈ 0.8 s
    p = Pose2D()
    for _ in range(n):
        p = p.integrate(v, omega, dt)

    # 1) 离散真值：每个采样精确加 ω·dt
    assert p.yaw == pytest.approx(n * omega * dt, abs=1e-12)
    # 2) 离散化误差在 ω·dt 量级内
    assert abs(p.yaw - math.pi / 2) <= omega * dt
    # 3) 终点落在半径 R 的圆弧上。
    #    左转的瞬时圆心在车的**左侧**，即 (0, R)，不是 (R, R) —— 起点在圆上，
    #    终点 (R,R) 也满足 x² + (y−R)² = R²，所以两者到圆心的距离都应是 R。
    assert math.hypot(p.x - 0.0, p.y - R) == pytest.approx(R, rel=1e-3)
    assert p.x == pytest.approx(R * math.sin(p.yaw), rel=1e-3)
    assert p.y == pytest.approx(R * (1 - math.cos(p.yaw)), rel=1e-3)


def test_pose_yaw_rate_is_exact_regardless_of_dt():
    """ω 是常量时，转角必须精确等于 ω·T，与 dt 怎么分无关。"""
    omega, T = 0.7, 2.3
    for dt in (0.05, 0.02, 0.11):
        p = Pose2D()
        n = int(T / dt)
        for _ in range(n):
            p = p.integrate(0.0, omega, dt)
        assert p.yaw == pytest.approx(wrap_angle(omega * n * dt), abs=1e-12)


def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-3 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(2 * math.pi) == pytest.approx(0.0, abs=1e-12)


def test_quaternion_from_yaw():
    x, y, z, w = quaternion_from_yaw(math.pi / 2)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sin(math.pi / 4))
    assert w == pytest.approx(math.cos(math.pi / 4))
    assert z * z + w * w == pytest.approx(1.0)


# ==========================================================================
# 0x02 MOVE 的反解与正解
# ==========================================================================
#
# 这一节钉住的是"哪些 (v, ω) 固件根本表达不出来"，以及**投影的方向**。
# 依据：chassis_motion.v:32-87 —— 内侧轮 `base×ratio>>8` 是**无符号**的，
# 只能缩小、不能反转、不能独立赋值。

def test_twist_to_move_straight_line():
    g = SkidSteerGeometry()
    mc = twist_to_move(0.10, 0.0, g)
    assert mc.motion_mode == 1                 # FORWARD
    assert mc.achievable
    assert mc.v == pytest.approx(0.10, abs=0.005)
    assert mc.omega == pytest.approx(0.0, abs=1e-9)


def test_twist_to_move_arc_uses_turn_modes():
    """左转 → mode 3 且内侧是左轮；右转 → mode 4 且内侧是右轮。"""
    g = SkidSteerGeometry()
    left = twist_to_move(0.15, +0.4, g)
    right = twist_to_move(0.15, -0.4, g)
    assert left.motion_mode == 3
    assert right.motion_mode == 4
    assert left.achievable and right.achievable
    # 内侧轮比外侧慢，但**同号**
    assert 0 <= left.turn_inner_ratio_q8 < 255
    assert 0 <= right.turn_inner_ratio_q8 < 255
    assert left.omega > 0 > right.omega


def test_arc_round_trip_within_reachable_region():
    """可达区内 (v, ω) → MOVE → (v, ω) 基本还原（差在 ratio 的 1/256 量化）。

    可达判据是 `R = v/ω ≥ L/2`，即 `|ω| ≤ 2v/L`。所以这里按 v 配 ω，
    不去测本来就该被投影掉的组合 —— 那由后面几条测试专门覆盖。
    """
    g = SkidSteerGeometry()
    for v in (0.05, 0.10, 0.20):
        limit = 2.0 * v / g.effective_separation_m
        for frac in (-0.95, -0.5, 0.0, 0.5, 0.95):
            omega = limit * frac
            mc = twist_to_move(v, omega, g)
            assert mc.achievable, f"({v},{omega}) 应当可达"
            assert mc.v == pytest.approx(v, abs=0.01)
            assert mc.omega == pytest.approx(omega, abs=0.03)


def test_tightest_reachable_turn_is_half_track_width():
    """`ratio=0`（内侧轮停转）就是最紧的弯：半径 = 有效轮距/2。

    这是"内侧轮不能反转"直接推出来的物理下限，也是必须让导航层知道的约束。
    """
    g = SkidSteerGeometry()
    mc = twist_to_move(0.10, 1.0, g)          # v/ω = 0.1 = L/2
    assert mc.motion_mode == 3
    assert mc.turn_inner_ratio_q8 == 0
    assert mc.achievable


def test_sharper_than_half_track_is_projected_to_spin():
    """半径 < L/2 需要内侧轮反转 → 投影成原地转，并且**必须**标记不可达。"""
    g = SkidSteerGeometry()
    mc = twist_to_move(0.05, 1.2, g)           # v/ω ≈ 0.042 < 0.1
    assert mc.motion_mode in (5, 6)
    assert not mc.achievable
    assert mc.note, '投影必须留下说明，不能静默吃掉 v'
    assert abs(mc.v) < 1e-9, '投影后应当是纯原地转'
    assert mc.omega > 0


def test_reverse_arc_is_projected_to_straight_backward_preserving_v():
    """固件没有"倒退转弯"这种动作（mode 2 只有直退）→ 丢掉 ω、**保留 v**。

    ★ 保留 v 而不是"按峰值轮速推算"很重要：后者会让车比命令的更快，
      方向是危险的。这条测试就是钉住这个选择。
    """
    g = SkidSteerGeometry()
    mc = twist_to_move(-0.10, 0.30, g)
    assert mc.motion_mode == 2
    assert not mc.achievable
    assert mc.v == pytest.approx(-0.10, abs=0.005), '投影后速度应当还是命令速度'
    assert mc.omega == pytest.approx(0.0, abs=1e-9)


def test_pure_spin_is_exact():
    g = SkidSteerGeometry()
    for omega in (0.5, -0.5):
        mc = twist_to_move(0.0, omega, g)
        assert mc.achievable
        assert mc.motion_mode == (5 if omega > 0 else 6)
        assert mc.omega == pytest.approx(omega, abs=0.02)


def test_over_rpm_limit_is_reported_not_silently_clamped():
    g = SkidSteerGeometry()
    mc = twist_to_move(0.9, 0.0, g)
    assert not mc.achievable
    assert mc.motion_mode == 0 and mc.base_rpm == 0
    assert '200' in mc.note


def test_move_to_twist_forward_and_backward():
    g = SkidSteerGeometry()
    v, omega = move_to_twist(1, 29, 255, g)
    assert v == pytest.approx(0.10, abs=0.005)
    assert omega == pytest.approx(0.0, abs=1e-9)
    v, omega = move_to_twist(2, 29, 255, g)
    assert v == pytest.approx(-0.10, abs=0.005)


def test_move_to_twist_matches_the_firmware_expansion():
    """正解必须走固件自己的展开逻辑，不能另写一套。

    否则驱动的 `achieved=` 诊断会与固件真实行为不符 —— 那种"日志说 A、
    车做 B"的偏差在真机上极难查。
    """
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from acg720_firmware import chassis_motion

    g = SkidSteerGeometry()
    wheels = chassis_motion(3, 100, 128)
    left = (wheels[1] + wheels[2]) / 2.0
    right = (wheels[0] + wheels[3]) / 2.0
    assert move_to_twist(3, 100, 128, g) == pytest.approx(
        g.wheels_to_twist(left, right))


def test_twist_to_move_rejects_bad_limit():
    with pytest.raises(ValueError):
        twist_to_move(0.1, 0.0, SkidSteerGeometry(), max_base_rpm=0)


# ==========================================================================
# 0x05 POSITION 的目标计数口径
# ==========================================================================
#
# ★ 口径来自 chassis_position_controller.v:78：
#       progress = (归一化四轮增量之和) >> 2
#   是**平均**不是求和。搞错的话标定出来的每米计数会差 4 倍。

def test_position_counts_for_half_metre_is_3232():
    """0.5 m ↔ 3232 计数。

    这不是巧合：交接包示例里的 `POSITION(1, 25, 3232)` 正好是 0.5 m。
    这个数从"1320 计数/圈 + 65 mm 轮径"独立算出来能对上，
    是对 CPR 与轮径这一组参数的一次反向印证。
    """
    g = SkidSteerGeometry()
    assert g.distance_to_position_counts(0.5) == pytest.approx(3232.1, abs=1.0)
    assert g.position_counts_to_distance_m(3232) == pytest.approx(0.5000, abs=0.001)


def test_position_counts_for_quarter_turn():
    """90° 原地转 ≈ 1015 计数（几何轮距、scale=1 时）。"""
    g = SkidSteerGeometry()
    assert g.rotation_to_position_counts(math.pi / 2) == pytest.approx(1015.4, abs=1.0)
    assert g.position_counts_to_rotation_rad(1015.4) == pytest.approx(math.pi / 2, abs=1e-3)


def test_position_rotation_uses_effective_separation():
    """原地转的计数必须随有效轮距变 —— 打滑越大，转同样的角要走更多计数。"""
    geo = SkidSteerGeometry(scale=1.0)
    skid = SkidSteerGeometry(scale=1.4)
    assert skid.rotation_to_position_counts(1.0) == pytest.approx(
        geo.rotation_to_position_counts(1.0) * 1.4, rel=1e-9)


def test_position_counts_round_trip():
    g = SkidSteerGeometry(scale=1.2)
    for metres in (0.0, 0.1, 0.5, 3.0):
        assert g.position_counts_to_distance_m(
            g.distance_to_position_counts(metres)) == pytest.approx(metres, abs=1e-9)
    for rad in (0.0, 0.5, math.pi, 2 * math.pi):
        assert g.position_counts_to_rotation_rad(
            g.rotation_to_position_counts(rad)) == pytest.approx(rad, abs=1e-9)
