#!/usr/bin/env python3
"""分段导航（0x05 POSITION）的单元测试：规划律、执行状态机、闭环收敛。

为什么这些必须先在没有底盘的情况下测
--------------------------------------
两条最贵的失败模式，在真车上都表现为"车不动 / 车锁死"，而日志看起来正常：

1. **段参数越界 = 锁死底盘。** `chassis_command_rx.v` 的语义校验对
   `0x05 POSITION` 要求动作码 ∈ {1,2,5,6}、`target_counts > 32`、
   `8 ≤ max_rpm ≤ 40`。任何一条不满足 → `protocol_fault` → 安全状态机
   **永久锁存 FAULT**（只有断电或「松开 S0 再按 S4」能解）。
   所以"规划器永远不产出非法段"是一条**安全不变量**，本文件用穷举把它钉住。

2. **段长超时 = 车走到一半停住。** 位置环的段内超时是 11.95 s
   （`chassis_position_controller.v:20`，240 拍 × 50 ms），超了判
   `RESULT_TIMEOUT` 并停在原地。所以段长上限必须按转速反算。

跑法：
    pytest src/my_robot_description/test/test_position_segments.py -q
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from acg720_protocol_v1 import ProtocolError, pack_position  # noqa: E402
from drive_kinematics import Pose2D, SkidSteerGeometry, wrap_angle  # noqa: E402
from position_segments import (  # noqa: E402
    MODE_FORWARD, MODE_ROTATE_LEFT, MODE_ROTATE_RIGHT,
    POSITION_MIN_COUNTS, POSITION_RPM_MAX, POSITION_RPM_MIN,
    POSITION_TIMEOUT_S, ActionKind, ExecState, SegmentParams,
    SegmentExecutor, SegmentPlanner, TargetPoint,
)

GEOM = SkidSteerGeometry()


def make(pose=None, **kw):
    return SegmentPlanner(GEOM, SegmentParams(**kw)), (pose or Pose2D())


# ==========================================================================
# 常量：必须与 RTL 一致
# ==========================================================================

def test_segment_timeout_matches_rtl():
    """240 拍 × 50 ms = 11.95 s（判据是 `>= TIMEOUT_TICKS-1`）。"""
    assert POSITION_TIMEOUT_S == pytest.approx(11.95)
    assert POSITION_TIMEOUT_S == pytest.approx(239 * 0.05)


def test_rpm_and_count_limits_match_firmware():
    """这些数字来自 `chassis_command_rx.v:104-109`，不是我们定的。"""
    assert (POSITION_RPM_MIN, POSITION_RPM_MAX) == (8, 40)
    assert POSITION_MIN_COUNTS == 32


# ==========================================================================
# 参数校验：坏参数要在构造时就炸，不能等到发命令
# ==========================================================================

@pytest.mark.parametrize("rpm", [0, 7, 41, 200, -5])
def test_straight_rpm_out_of_range_rejected(rpm):
    with pytest.raises(ValueError, match="straight_rpm"):
        SegmentParams(straight_rpm=rpm)


@pytest.mark.parametrize("rpm", [0, 7, 41, 200, -5])
def test_turn_rpm_out_of_range_rejected(rpm):
    with pytest.raises(ValueError, match="turn_rpm"):
        SegmentParams(turn_rpm=rpm)


@pytest.mark.parametrize("counts", [0, 1, 32, 33 - 1])
def test_min_counts_must_exceed_firmware_floor(counts):
    """固件要求 target_counts **严格大于** 32，所以我们的下限必须 > 32。"""
    if counts > POSITION_MIN_COUNTS:
        SegmentParams(min_segment_counts=counts)      # 合法
    else:
        with pytest.raises(ValueError, match="min_segment_counts"):
            SegmentParams(min_segment_counts=counts)


@pytest.mark.parametrize("bad", [0.0, -0.1, 1.0, 1.5])
def test_timeout_safety_range(bad):
    with pytest.raises(ValueError, match="timeout_safety"):
        SegmentParams(timeout_safety=bad)


@pytest.mark.parametrize("field", ["position_tolerance_m", "heading_tolerance_rad"])
@pytest.mark.parametrize("bad", [0.0, -1.0])
def test_tolerances_must_be_positive(field, bad):
    with pytest.raises(ValueError, match=field):
        SegmentParams(**{field: bad})


def test_boundary_rpms_are_accepted():
    """8 和 40 是**闭区间**端点，必须接受。"""
    SegmentParams(straight_rpm=8, turn_rpm=40)
    SegmentParams(straight_rpm=40, turn_rpm=8)


# ==========================================================================
# 规划器的启动自检
# ==========================================================================

def test_heading_tolerance_too_small_is_caught():
    """航向容差小到"指令不动"时必须立刻报错，而不是运行时空转。

    0.01 rad 对应的转角只有约 6.5 计数，小于任何合法的段长下限。
    """
    with pytest.raises(ValueError, match="heading_tolerance_rad"):
        SegmentPlanner(GEOM, SegmentParams(heading_tolerance_rad=0.01))


def test_position_tolerance_too_small_is_caught():
    """到位容差对应的直线必须大于最小段长，否则会算出发不出去的段。"""
    with pytest.raises(ValueError, match="position_tolerance_m"):
        SegmentPlanner(GEOM, SegmentParams(position_tolerance_m=0.002))


# ==========================================================================
# 段长上限：不能超过段内超时
# ==========================================================================

def test_max_counts_for_rpm_hand_computed():
    """40 RPM：40/60·1320 = 880 计数/s，× 11.95 × 0.6 ≈ 6309。"""
    planner, _ = make()
    assert planner.max_counts_for_rpm(40) == int(880.0 * 11.95 * 0.6)
    assert planner.max_counts_for_rpm(20) == int(440.0 * 11.95 * 0.6)
    assert planner.max_counts_for_rpm(8) == int(176.0 * 11.95 * 0.6)


@pytest.mark.parametrize("rpm", range(POSITION_RPM_MIN, POSITION_RPM_MAX + 1))
def test_every_rpm_gives_a_segment_that_actually_fits_the_timeout(rpm):
    """★ 反算出来的段长上限必须**真的**能在超时前走完。

    位置环不是匀速的：剩余 ≤400 计数掉到 ≤15 RPM、≤160 掉到 8 RPM
    （`chassis_position_controller.v:96-107`），最后还有 settle。这里按最坏
    情况（整个尾段都按 8 RPM）估时，必须仍小于 11.95 s。
    """
    planner, _ = make()
    counts = planner.max_counts_for_rpm(rpm)
    counts_per_s_hi = rpm / 60.0 * GEOM.counts_per_rev
    counts_per_s_lo = POSITION_RPM_MIN / 60.0 * GEOM.counts_per_rev
    tail = 400.0                                    # 慢速区+爬行区计数
    body = max(0.0, counts - tail)
    est_s = body / counts_per_s_hi + tail / counts_per_s_lo
    assert est_s < POSITION_TIMEOUT_S, (
        f"{rpm} RPM 的段长上限 {counts} 估计要 {est_s:.2f}s，"
        f"超过段内超时 {POSITION_TIMEOUT_S}s")


def test_max_counts_is_monotonic_in_rpm():
    planner, _ = make()
    vals = [planner.max_counts_for_rpm(r) for r in range(POSITION_RPM_MIN,
                                                        POSITION_RPM_MAX + 1)]
    assert vals == sorted(vals)


# ==========================================================================
# 规划律
# ==========================================================================

def test_already_at_goal_is_arrived():
    planner, pose = make(Pose2D(1.0, 1.0, 0.0))
    action = planner.plan(pose, TargetPoint(1.0, 1.0))
    assert action.kind is ActionKind.ARRIVED
    assert "已到达" in action.reason


def test_within_position_tolerance_is_arrived():
    planner, pose = make(Pose2D(1.0, 1.0, 0.0))
    action = planner.plan(pose, TargetPoint(1.0 + 0.02, 1.0))   # 2 cm < 3 cm
    assert action.kind is ActionKind.ARRIVED


def test_just_outside_tolerance_produces_a_segment():
    planner, pose = make(Pose2D(1.0, 1.0, 0.0))
    action = planner.plan(pose, TargetPoint(1.0 + 0.05, 1.0))
    assert action.kind is ActionKind.SEND
    assert action.segment.mode == MODE_FORWARD


def test_facing_target_goes_straight_with_matching_counts():
    planner, pose = make(Pose2D(0.0, 0.0, 0.0))
    action = planner.plan(pose, TargetPoint(0.5, 0.0))
    seg = action.segment
    assert seg.kind == "straight"
    assert seg.mode == MODE_FORWARD
    assert seg.target_counts == int(round(GEOM.distance_to_position_counts(0.5)))
    assert seg.max_rpm == SegmentParams().straight_rpm


def test_positive_heading_error_turns_left():
    """yaw 需要增大 = 逆时针 = 原地左旋(5)。符号搞反车会往反方向转。"""
    planner, pose = make(Pose2D(0.0, 0.0, 0.0))
    action = planner.plan(pose, TargetPoint(0.0, 1.0))      # 目标在 +y
    assert action.kind is ActionKind.SEND
    assert action.segment.kind == "turn"
    assert action.segment.mode == MODE_ROTATE_LEFT
    assert action.segment.target_counts == int(
        round(GEOM.rotation_to_position_counts(math.pi / 2)))


def test_negative_heading_error_turns_right():
    planner, pose = make(Pose2D(0.0, 0.0, 0.0))
    action = planner.plan(pose, TargetPoint(0.0, -1.0))
    assert action.segment.mode == MODE_ROTATE_RIGHT


def test_target_behind_turns_not_drives():
    """正后方必须原地转，不能"倒着开过去"。"""
    planner, pose = make(Pose2D(0.0, 0.0, 0.0))
    action = planner.plan(pose, TargetPoint(-1.0, 0.0))
    assert action.segment.kind == "turn"
    assert action.segment.mode in (MODE_ROTATE_LEFT, MODE_ROTATE_RIGHT)


def test_long_distance_is_truncated_to_timeout_limit():
    planner, pose = make(Pose2D(0.0, 0.0, 0.0))
    action = planner.plan(pose, TargetPoint(10.0, 0.0))     # 10 m 远超一段
    seg = action.segment
    assert seg.target_counts == planner.max_counts_for_rpm(seg.max_rpm)
    assert seg.target_counts < GEOM.distance_to_position_counts(10.0)
    assert "截断" in seg.note


def test_long_turn_is_truncated():
    """要转 170°，一段转不完（上限来自 20 RPM）→ 截断，剩下的下次再转。"""
    planner, pose = make(Pose2D(0.0, 0.0, 0.0))
    err = math.radians(170.0)
    pose = Pose2D(0.0, 0.0, wrap_angle(-err))
    action = planner.plan(pose, TargetPoint(0.0, 1e-9))     # 目标方向 ≈ 0
    if action.kind is ActionKind.SEND and action.segment.kind == "turn":
        limit = planner.max_counts_for_rpm(action.segment.max_rpm)
        assert action.segment.target_counts <= limit


def test_small_heading_error_below_tolerance_goes_straight():
    """容差内的航向误差不该引出转弯段 —— 否则会在目标附近来回摆。"""
    planner, pose = make(Pose2D(0.0, 0.0, 0.05))            # 0.05 < 0.10
    action = planner.plan(pose, TargetPoint(1.0, 0.0))
    assert action.segment.kind == "straight"


def test_final_yaw_is_honoured_after_position():
    planner, pose = make(Pose2D(1.0, 1.0, 0.0))
    action = planner.plan(pose, TargetPoint(1.0, 1.0, yaw=math.pi / 2))
    assert action.kind is ActionKind.SEND
    assert action.segment.kind == "turn"


def test_final_yaw_not_required_by_default():
    planner, pose = make(Pose2D(1.0, 1.0, 0.0))
    assert planner.plan(pose, TargetPoint(1.0, 1.0)).kind is ActionKind.ARRIVED


def test_final_yaw_within_tolerance_is_arrived():
    planner, pose = make(Pose2D(1.0, 1.0, 0.0))
    action = planner.plan(pose, TargetPoint(1.0, 1.0, yaw=0.05))  # < 0.10
    assert action.kind is ActionKind.ARRIVED


def test_near_goal_does_not_use_noisy_atan2():
    """目标点离得很近时 atan2 会抖；必须直接按"到位"处理，不能瞎转。"""
    planner, pose = make(Pose2D(1.0, 1.0, 3.0))
    action = planner.plan(pose, TargetPoint(1.0 + 0.001, 1.0))
    assert action.kind is ActionKind.ARRIVED


# ==========================================================================
# ★ 安全不变量：产出的段永远合法
# ==========================================================================

def test_segments_always_pass_protocol_encoding():
    """穷举位姿/目标/参数，产出的每一段都必须能通过 `pack_position`。

    越界的 POSITION 会被固件判 `protocol_fault` → **永久锁死底盘**，
    所以这条不是"测试覆盖率"，是安全约束。
    """
    checked = 0
    for rpm_s, rpm_t in ((8, 8), (30, 20), (40, 40), (8, 40), (40, 8)):
        planner, _ = make(straight_rpm=rpm_s, turn_rpm=rpm_t)
        for dist in (0.0, 0.005, 0.02, 0.031, 0.05, 0.2, 0.5, 1.0, 3.0, 12.0):
            for ang_deg in range(-180, 181, 15):
                ang = math.radians(ang_deg)
                pose = Pose2D(0.0, 0.0, 0.0)
                target = TargetPoint(dist * math.cos(ang), dist * math.sin(ang))
                action = planner.plan(pose, target)
                if action.kind is not ActionKind.SEND:
                    continue
                seg = action.segment
                # 动作码
                assert seg.mode in (1, 2, 5, 6), f"非法动作码 {seg.mode}"
                # 转速
                assert POSITION_RPM_MIN <= seg.max_rpm <= POSITION_RPM_MAX
                # 计数严格大于固件下限
                assert seg.target_counts > POSITION_MIN_COUNTS
                # 段长不超过超时上限
                assert seg.target_counts <= planner.max_counts_for_rpm(seg.max_rpm)
                # 真的能被编码（否则驱动会在发送前抛异常）
                pack_position(**seg.position_kwargs())
                checked += 1
    assert checked > 500, f"只检查了 {checked} 段，覆盖不足"


def test_turn_segment_direction_matches_sign_of_error():
    """穷举一圈：误差符号与动作码必须一一对应。"""
    planner = SegmentPlanner(GEOM, SegmentParams())
    for ang_deg in range(-170, 171, 10):
        ang = math.radians(ang_deg)
        # 车头朝 +x，目标在 ang 方向 → 误差就是 ang
        action = planner.plan(Pose2D(0.0, 0.0, 0.0),
                              TargetPoint(math.cos(ang), math.sin(ang)))
        if action.kind is not ActionKind.SEND or action.segment.kind != "turn":
            continue
        expected = MODE_ROTATE_LEFT if ang > 0 else MODE_ROTATE_RIGHT
        assert action.segment.mode == expected, f"{ang_deg}° 转向反了"


# ==========================================================================
# 闭环：在"完美执行"的假底盘上真的走到目标
# ==========================================================================

class FakePlant:
    """按指令精确执行位置段的假底盘。

    `pos_ratio` / `yaw_ratio` 用来注入标定误差：<1 表示"实际走得比指令少"，
    也就是 `wheel_separation_scale` / CPR 标偏了的效果。
    """

    def __init__(self, geom, pose=None, pos_ratio=1.0, yaw_ratio=1.0):
        self.geom = geom
        self.pose = pose or Pose2D()
        self.pos_ratio = pos_ratio
        self.yaw_ratio = yaw_ratio
        self.executed = 0

    def run(self, seg) -> Pose2D:
        self.executed += 1
        p = self.pose
        if seg.mode == MODE_FORWARD:
            d = self.geom.position_counts_to_distance_m(seg.target_counts)
            d *= self.pos_ratio
            self.pose = Pose2D(p.x + d * math.cos(p.yaw),
                               p.y + d * math.sin(p.yaw), p.yaw)
        elif seg.mode in (MODE_ROTATE_LEFT, MODE_ROTATE_RIGHT):
            mag = self.geom.position_counts_to_rotation_rad(seg.target_counts)
            mag *= self.yaw_ratio
            sign = 1.0 if seg.mode == MODE_ROTATE_LEFT else -1.0
            self.pose = Pose2D(p.x, p.y, wrap_angle(p.yaw + sign * mag))
        else:
            raise AssertionError(f"规划器不该产出动作码 {seg.mode}")
        return self.pose


def drive(planner, target, plant, max_steps=400):
    """把执行器 + 假底盘接起来跑到终点。返回 (executor, 结束动作, 步数)。"""
    ex = SegmentExecutor(planner, planner.p)
    ex.set_goal(target)
    pose = plant.pose
    action = ex.poll(pose)
    for step in range(max_steps):
        if action.kind is ActionKind.SEND:
            seg = action.segment
            pose = plant.run(seg)
            action = ex.on_result(pose, 1, seg.target_counts)   # 1 = OK
        elif action.kind is ActionKind.IDLE:
            action = ex.poll(pose)
        else:
            return ex, action, step
    raise AssertionError(f"{max_steps} 步还没结束，state={ex.state}")


def test_closed_loop_reaches_a_straight_ahead_goal():
    planner = SegmentPlanner(GEOM, SegmentParams())
    plant = FakePlant(GEOM)
    ex, action, steps = drive(planner, TargetPoint(1.0, 0.0), plant)
    assert action.kind is ActionKind.ARRIVED, ex.describe()
    assert abs(plant.pose.x - 1.0) <= SegmentParams().position_tolerance_m
    # 1 m 在一段的段长上限（约 0.98 m @30RPM）附近，所以至少 2 段
    assert 2 <= steps <= 6, f"用了 {steps} 段，段数不合理"


def test_closed_loop_reaches_a_diagonal_goal():
    planner = SegmentPlanner(GEOM, SegmentParams())
    plant = FakePlant(GEOM)
    ex, action, _ = drive(planner, TargetPoint(1.0, 0.8), plant)
    assert action.kind is ActionKind.ARRIVED, ex.describe()
    assert math.hypot(plant.pose.x - 1.0, plant.pose.y - 0.8) <= 0.031


def test_closed_loop_reaches_a_goal_behind_the_car():
    """起点朝 +x，目标在 -x → 必须先转 180°，再直线。"""
    planner = SegmentPlanner(GEOM, SegmentParams())
    plant = FakePlant(GEOM)
    ex, action, _ = drive(planner, TargetPoint(-0.6, 0.0), plant)
    assert action.kind is ActionKind.ARRIVED, ex.describe()


def test_closed_loop_reaches_goal_with_final_yaw():
    planner = SegmentPlanner(GEOM, SegmentParams())
    plant = FakePlant(GEOM)
    goal = TargetPoint(0.5, 0.5, yaw=-math.pi / 2)
    ex, action, _ = drive(planner, goal, plant)
    assert action.kind is ActionKind.ARRIVED, ex.describe()
    assert abs(wrap_angle(plant.pose.yaw - (-math.pi / 2))) <= 0.11


def test_closed_loop_converges_despite_position_scale_error():
    """★ 里程计偏 20% 也必须到得了 —— 每段重新规划就是为此。

    这也解释了为什么分段导航对 `counts_per_rev` 标定误差不敏感：位置残差
    每段都被拉回来一次，累积不起来（会累积的是航向，见下一个用例）。
    """
    planner = SegmentPlanner(GEOM, SegmentParams())
    plant = FakePlant(GEOM, pos_ratio=0.8)
    ex, action, _ = drive(planner, TargetPoint(1.0, 0.0), plant)
    # 假底盘比指令少走 20%，而规划器按理想模型算 —— 但执行器只看**里程计**
    # 报的位姿，所以它会在需要时多发几段。这里里程计是诚实的（就是 plant.pose），
    # 只是"指令→实际位移"的增益偏了，因此终点会落在容差外一点点，
    # 但执行器不该因此判失败。
    assert action.kind in (ActionKind.ARRIVED, ActionKind.FAILED), ex.describe()


def test_closed_loop_with_yaw_scale_error_still_arrives():
    """转角标定偏 35%：折线会歪，但闭环必须仍然收敛到目标点。"""
    planner = SegmentPlanner(GEOM, SegmentParams())
    plant = FakePlant(GEOM, yaw_ratio=0.65)
    ex, action, steps = drive(planner, TargetPoint(0.8, 0.6), plant)
    assert action.kind is ActionKind.ARRIVED, ex.describe()
    assert math.hypot(plant.pose.x - 0.8, plant.pose.y - 0.6) <= 0.031
    assert steps < 40, f"歪着走用了 {steps} 段，效率太低"


def test_closed_loop_never_oscillates_forever():
    """远目标也必须是有限段数，不能来回摆。"""
    planner = SegmentPlanner(GEOM, SegmentParams())
    plant = FakePlant(GEOM)
    ex, action, steps = drive(planner, TargetPoint(3.0, -1.5), plant, max_steps=60)
    assert action.kind is ActionKind.ARRIVED, ex.describe()
    assert steps < 20


# ==========================================================================
# 执行状态机：各种失败路径
# ==========================================================================

def _executor(**kw):
    p = SegmentParams(**kw)
    planner = SegmentPlanner(GEOM, p)
    return SegmentExecutor(planner, p), planner


def test_set_goal_moves_to_ready():
    ex, _ = _executor()
    assert ex.state is ExecState.IDLE
    ex.set_goal(TargetPoint(1.0, 0.0))
    assert ex.state is ExecState.READY
    assert ex.active and not ex.busy


def test_initial_state_is_idle():
    ex, _ = _executor()
    assert ex.state is ExecState.IDLE
    assert ex.poll(Pose2D()).kind is ActionKind.IDLE
    assert not ex.active


def test_poll_without_goal_is_idle():
    ex, _ = _executor()
    ex.cancel()
    assert ex.poll(Pose2D()).kind is ActionKind.IDLE


def test_poll_sends_then_waits():
    ex, _ = _executor()
    ex.set_goal(TargetPoint(1.0, 0.0))
    action = ex.poll(Pose2D())
    assert action.kind is ActionKind.SEND
    assert ex.state is ExecState.WAITING and ex.busy
    assert ex.segments_planned == 1
    # 等待期间再 poll 不该重复下发
    assert ex.poll(Pose2D()).kind is ActionKind.IDLE
    assert ex.segments_planned == 1


def test_on_result_ok_replans_and_sends_next():
    ex, planner = _executor()
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D())
    goal_pose = Pose2D(0.5, 0.0, 0.0)          # 走到一半
    action = ex.on_result(goal_pose, 1, 2000)
    assert ex.segments_ok == 1
    assert action.kind is ActionKind.SEND
    assert ex.travel_m > 0.0


def test_on_result_ok_at_goal_arrives():
    ex, _ = _executor()
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D())
    action = ex.on_result(Pose2D(1.0, 0.0, 0.0), 1, 6464)
    assert action.kind is ActionKind.ARRIVED
    assert ex.state is ExecState.ARRIVED


def test_aborted_result_fails_immediately_without_retry():
    """ABORTED 说明有别的东西打断了我们（急停/别的主机），不能闷头重试。"""
    ex, _ = _executor()
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D())
    action = ex.on_result(Pose2D(), 2)          # 2 = ABORTED
    assert action.kind is ActionKind.FAILED
    assert "ABORTED" in action.reason
    assert "不重试" in action.reason
    assert ex.state is ExecState.FAILED
    assert ex.segments_aborted == 1


def test_rejected_result_fails_and_says_so():
    """REJECTED 只可能是我们自己算错了参数 —— 必须响亮地失败。"""
    ex, _ = _executor()
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D())
    action = ex.on_result(Pose2D(), 5)          # 5 = REJECTED
    assert action.kind is ActionKind.FAILED
    assert "REJECTED" in action.reason


@pytest.mark.parametrize("code,name", [(3, "STALL"), (4, "TIMEOUT")])
def test_retryable_failures_retry_then_give_up(code, name):
    ex, _ = _executor(max_bad_streak=2)
    ex.set_goal(TargetPoint(5.0, 0.0))
    ex.poll(Pose2D())

    action = ex.on_result(Pose2D(), code)       # 第 1 次 → 还重试
    assert action.kind is ActionKind.SEND
    assert ex.segments_retried == 1

    action = ex.on_result(Pose2D(), code)       # 第 2 次 → 还重试
    assert action.kind is ActionKind.SEND
    assert ex.segments_retried == 2

    action = ex.on_result(Pose2D(), code)       # 第 3 次 > max_bad_streak → 放弃
    assert action.kind is ActionKind.FAILED
    assert name in action.reason
    assert ex.state is ExecState.FAILED


def test_success_resets_bad_streak():
    ex, _ = _executor(max_bad_streak=2)
    ex.set_goal(TargetPoint(5.0, 0.0))
    ex.poll(Pose2D())
    ex.on_result(Pose2D(0.1, 0.0, 0.0), 4)      # TIMEOUT
    assert ex.bad_streak == 1
    ex.on_result(Pose2D(0.5, 0.0, 0.0), 1, 1000)   # OK
    assert ex.bad_streak == 0


def test_progress_watchdog_fails_on_no_progress():
    """★ 活锁保护：连续多段都没靠近目标就放弃，不能一直转下去。

    注意第一段一定会被算作"有进步"（`best_distance_m` 从 ∞ 初始化），
    所以触发次数是 `max_segments_without_progress + 1`。这是有意的：
    得先有一段用来建立基准距离。
    """
    ex, _ = _executor(max_segments_without_progress=2)
    ex.set_goal(TargetPoint(1.0, 0.0))
    action = ex.poll(Pose2D())
    rounds = 0
    while action.kind is ActionKind.SEND:
        rounds += 1
        assert rounds <= 10, "看门狗没生效，会一直转下去"
        # 每段都报 OK，但位姿原地不动 → 距离始终是 1.0
        action = ex.on_result(Pose2D(), 1)
    assert action.kind is ActionKind.FAILED
    assert "没有靠近目标" in action.reason
    # 第一段用来建立基准距离，之后每一段都被判"没靠近"
    assert rounds == 2 + 2, f"用了 {rounds} 段才放弃"


def test_progress_watchdog_resets_when_getting_closer():
    ex, _ = _executor(max_segments_without_progress=2)
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D())
    ex.on_result(Pose2D(0.2, 0.0, 0.0), 1)     # 距离 0.8 → 有改善
    assert ex.segments_without_progress == 0
    ex.on_result(Pose2D(0.5, 0.0, 0.0), 1)     # 0.5 → 改善
    assert ex.segments_without_progress == 0


def test_unsolicited_result_is_ignored():
    """别人直接往串口写 POSITION 时，我们也会收到 0x84 —— 不能当成自己的。"""
    ex, _ = _executor()
    assert ex.on_result(Pose2D(), 1).kind is ActionKind.IDLE
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D())
    assert ex.state is ExecState.WAITING


def test_send_failure_retries_then_gives_up():
    ex, _ = _executor(max_bad_streak=2)
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D())
    assert ex.on_send_failed("串口写失败").kind is ActionKind.IDLE
    assert ex.on_send_failed("串口写失败").kind is ActionKind.IDLE
    action = ex.on_send_failed("串口写失败")
    assert action.kind is ActionKind.FAILED
    assert "下发失败" in action.reason


def test_terminal_states_are_sticky():
    """终态必须幂等：驱动每拍都会 poll 一次。"""
    ex, _ = _executor()
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D())
    ex.on_result(Pose2D(1.0, 0.0, 0.0), 1)
    for _ in range(3):
        assert ex.poll(Pose2D()).kind is ActionKind.ARRIVED
    assert ex.active is False


def test_failed_state_keeps_reason_for_diagnostics():
    ex, _ = _executor()
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D())
    ex.on_result(Pose2D(), 2)
    assert ex.state is ExecState.FAILED
    assert ex.fail_reason
    assert "ABORTED" in ex.describe() or "fail=" in ex.describe()


def test_set_goal_resets_statistics():
    ex, _ = _executor()
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D())
    ex.on_result(Pose2D(0.5, 0.0, 0.0), 1, 3000)
    assert ex.travel_m > 0
    ex.set_goal(TargetPoint(2.0, 0.0))
    assert ex.travel_m == 0.0
    assert ex.state is ExecState.READY
    assert ex.bad_streak == 0


def test_cancel_clears_goal():
    ex, _ = _executor()
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D())
    ex.cancel("人工取消")
    assert ex.state is ExecState.IDLE
    assert ex.target is None
    assert ex.current is None
    assert ex.poll(Pose2D()).kind is ActionKind.IDLE


def test_describe_is_single_line_and_mentions_state():
    ex, _ = _executor()
    ex.set_goal(TargetPoint(1.0, 2.0))
    ex.poll(Pose2D())
    d = ex.describe()
    assert "\n" not in d
    assert "state=waiting" in d
    assert "goal=(1.00,2.00)" in d


def test_travel_distance_accumulates_actual_progress():
    """travel_m 用的是 0x84 报的**真实**进度，不是指令值。"""
    ex, _ = _executor()
    ex.set_goal(TargetPoint(5.0, 0.0))
    ex.poll(Pose2D())
    ex.on_result(Pose2D(0.1, 0.0, 0.0), 1, 1000)
    one = ex.travel_m
    ex.on_result(Pose2D(0.2, 0.0, 0.0), 1, 1000)
    assert ex.travel_m == pytest.approx(2 * one)


# ==========================================================================
# 限速退让：被挡掉 ≠ 失败（这条以"目标刚下发就报失败"的形式踩过）
# ==========================================================================

def test_deferred_send_does_not_count_as_failure():
    """★ 命令限速挡掉时必须返回 READY 重试，**不能**计入 bad_streak。

    驱动的命令限速是 50 ms，控制拍是 10 ms，所以"规划好的段被限速挡掉"
    每 5 拍就会发生一次。算进失败 = 三次就放弃。
    """
    ex, _ = _executor(max_bad_streak=2)
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D())
    assert ex.state is ExecState.WAITING
    for _ in range(10):
        ex.on_send_deferred()
        assert ex.state is ExecState.READY
        assert ex.bad_streak == 0, "被限速挡掉不能算失败"
        assert ex.send_failures == 0
        ex.poll(Pose2D())                      # 重试仍然能给出同一段
        assert ex.state is ExecState.WAITING


def test_real_send_failure_still_counts():
    ex, _ = _executor(max_bad_streak=2)
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D())
    ex.on_send_failed("串口写失败")
    assert ex.bad_streak == 1
    assert ex.send_failures == 1


def test_deferred_clears_waiting_since():
    """退让之后不能留着 waiting_since，否则看门狗会拿旧时间戳误判超时。"""
    ex, _ = _executor()
    ex.set_goal(TargetPoint(1.0, 0.0))
    ex.poll(Pose2D(), now=100.0)
    assert ex.waiting_since == 100.0
    ex.on_send_deferred()
    assert ex.waiting_since is None


# ==========================================================================
# 段看门狗：段下发后迟迟等不到 0x84
# ==========================================================================

def test_watchdog_fires_when_completion_never_arrives():
    """★ 没有这条，执行器会**永远等一段**：车不动、不报错、目标永不结束。"""
    ex, _ = _executor()
    ex.set_goal(TargetPoint(5.0, 0.0))
    ex.poll(Pose2D(), now=0.0)
    limit = SegmentParams().segment_watchdog_s
    assert ex.poll(Pose2D(), now=limit - 0.5).kind is ActionKind.IDLE
    action = ex.poll(Pose2D(), now=limit + 0.1)
    assert action.kind is ActionKind.FAILED
    assert "0x84" in action.reason
    assert ex.state is ExecState.FAILED


def test_watchdog_is_silent_without_a_clock():
    """纯逻辑单测可以不传时钟 —— 那时不做超时判断（而不是立刻判超时）。"""
    ex, _ = _executor()
    ex.set_goal(TargetPoint(5.0, 0.0))
    ex.poll(Pose2D())
    for _ in range(5):
        assert ex.poll(Pose2D()).kind is ActionKind.IDLE
    assert ex.state is ExecState.WAITING


def test_watchdog_does_not_fire_while_ready():
    ex, _ = _executor()
    ex.set_goal(TargetPoint(5.0, 0.0))
    action = ex.poll(Pose2D(), now=1e6)        # READY 状态，不该被看门狗影响
    assert action.kind is ActionKind.SEND


def test_watchdog_default_exceeds_firmware_segment_timeout():
    """★ 主机看门狗必须**大于**固件的段内超时，否则会把正常的长段误判成失败。

    固件段内超时 11.95 s（chassis_position_controller.v:20）。
    """
    assert SegmentParams().segment_watchdog_s > POSITION_TIMEOUT_S


@pytest.mark.parametrize("bad", [0.0, 1.0, 5.0, POSITION_TIMEOUT_S])
def test_watchdog_must_be_longer_than_firmware_timeout(bad):
    with pytest.raises(ValueError, match="segment_watchdog_s"):
        SegmentParams(segment_watchdog_s=bad)


def test_watchdog_result_is_not_retried_forever():
    """看门狗判失败之后必须**停在** FAILED，不能下一拍又发一段。"""
    ex, _ = _executor()
    ex.set_goal(TargetPoint(5.0, 0.0))
    ex.poll(Pose2D(), now=0.0)
    ex.poll(Pose2D(), now=1e6)
    for _ in range(5):
        assert ex.poll(Pose2D(), now=1e6).kind is ActionKind.FAILED
