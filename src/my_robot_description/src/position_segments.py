#!/usr/bin/env python3
"""基于 0x05 POSITION 的分段导航（4A）：纯逻辑，不依赖 ROS。

为什么需要它
------------
当前固件有一条 3.0 s 的连续运动上限（`chassis_safety_fsm.v:11,95-97`）：
`0x02 MOVE` 连续跑满 3 秒就锁存 `FAULT_RUN_LIMIT`，而 `run_age` **不会**被
重发命令清零（只在离开 RUNNING 时清，`:74-77`）。所以连续闭环导航在现有
位流上做不了。

但位置环**不受**这条限制：

    // chassis_safety_fsm.v:57-58
    assign run_limit_reached = !position_active &&
                               (run_age >= RUN_LIMIT_CYCLES - 32'd1);

于是"把长距离拆成一串 `0x05 POSITION` 段"就能在不改固件的前提下连续移动。
本模块负责这个拆分，以及段与段之间的决策。

它做不到什么（都是从 RTL 推出来的硬约束，不是设计取舍）
--------------------------------------------------------
1. **没有圆弧。** 位置环只认四个动作码（`chassis_position_controller.v:44-47`，
   经 `chassis_command_rx.v:104-106` 校验）：前进 / 后退 / 原地左旋 / 原地右旋。
   所以轨迹只能是"转弯 + 直线"的**折线**，无法跟踪圆弧。要走曲线只能靠
   "缩短直线段 + 频繁小幅修正航向"去逼近。
2. **段运行期间不能抢占。** 启动条件是 `start_valid && !busy`（`:177`），
   运行中再发 POSITION 既不会重启也不会中止 —— 必须等 `done`（0x84）再发下一段。
3. **段内硬超时 11.95 s。** `TIMEOUT_TICKS=240` × 50 ms（`:20`）→
   `RESULT_TIMEOUT`。所以段长必须按 rpm 反算上限，见 `max_counts_for_rpm`。
4. **堵转判据**：连续 10 拍（500 ms）进展 ≤ 1 计数 → `RESULT_STALL`（`:241`）。
   低速段不会误判（8 RPM 每拍也有约 8.8 计数）。
5. **段进行中只能发 HEARTBEAT 或 POSITION。** `top.v:121-125` 里
   `position_abort` 认 STOP(0)/MOVE(2)/DISARM(3)/OPEN_LOOP(4) 这四种 kind，
   发出去就把当前段**中断**掉。HEARTBEAT(6) 和 ARM(1) 不会。
   另外 `top.v:152` 是 `target_rpm = position_busy ? position_target_rpm :
   manual_target_rpm` —— 段一结束 `busy` 落回 0，**残留的手动目标会立刻接管**。

精度
----
`STOP_TOL_COUNTS = 32`（`:19`）→ 每段的位置残差约 ±32 计数 ≈ 5 mm。
但每段结束后都用里程计**重新规划**，所以位置误差不会累积成"越走越偏"，
而是每段被拉回来一次。真正会累积的是**航向**：转角换算依赖
`effective_separation_m`，也就是 `wheel_separation_scale` 这个标定量。
标不准 → 每段都差一点 → 整条折线偏斜。所以用它之前先把
`wheel_separation_scale` 标出来（见 docs/real_robot_plan.md 第 3 节）。

用法（上位机侧）
----------------
    planner  = SegmentPlanner(geom, SegmentParams())
    executor = SegmentExecutor(planner, SegmentParams())
    executor.set_goal(TargetPoint(1.0, 0.5, yaw=0.0))

    action = executor.poll(pose)            # 空闲 → 给出下一段
    if action.kind is ActionKind.SEND:
        send_position(**action.segment.position_kwargs())

    # 收到 0x84 done=1 时
    action = executor.on_result(pose, result_code, progress_counts)
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from enum import Enum
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drive_kinematics import Pose2D, SkidSteerGeometry, wrap_angle  # noqa: E402

# ============================================================================
# 固件真值（不要在别处再写一遍这些数字）
# ============================================================================

#: `chassis_position_controller.v:20` —— 段内超时拍数
POSITION_TIMEOUT_TICKS = 240
#: 位置环与编码器测速共用的采样周期（50 ms）
POSITION_SAMPLE_TICK_S = 0.05
#: 段内硬超时。注意判据是 `timeout_counter >= TIMEOUT_TICKS - 1`（`:241`）
POSITION_TIMEOUT_S = (POSITION_TIMEOUT_TICKS - 1) * POSITION_SAMPLE_TICK_S

#: `chassis_position_controller.v:19` —— 剩余 ≤ 它就算到位（32 计数 ≈ 4.9 mm）
POSITION_STOP_TOL_COUNTS = 32
#: `:24` —— 慢速区
POSITION_SLOW_ZONE_COUNTS = 400
#: `:25` —— 爬行区
POSITION_CREEP_ZONE_COUNTS = 160

#: 动作码（`chassis_motion.v:20-26`，0x05 只接受这四种）
MODE_FORWARD = 1
MODE_BACKWARD = 2
MODE_ROTATE_LEFT = 5
MODE_ROTATE_RIGHT = 6

#: 0x05 参数范围（`chassis_command_rx.v:104-109`）
POSITION_RPM_MIN = 8
POSITION_RPM_MAX = 40
#: target_counts 必须**严格大于**它（`:108`）
POSITION_MIN_COUNTS = 32

#: 0x84 的 result_code 里，哪几个值得重试
RETRYABLE_RESULTS = frozenset({3, 4})       # STALL / TIMEOUT


# ============================================================================
# 参数
# ============================================================================


@dataclass(frozen=True)
class SegmentParams:
    """分段策略的参数。默认值是给 65 mm 轮、0.2 m 轮距的小车调过的。"""

    #: 航向误差大于它就原地转（弧度）。0.10 rad ≈ 5.7°
    #: 取值下限受**计数分辨率**约束：见 SegmentPlanner 的启动自检。
    heading_tolerance_rad: float = 0.10
    #: 离目标点小于它就认为到位（米）。要明显大于 STOP_TOL 的 5 mm 残差。
    position_tolerance_m: float = 0.03
    #: 小于这个计数的段不值得发：固件要求 > 32，这里留一点余量
    min_segment_counts: int = 40
    #: 直线段用多大转速。越大越快，但太大会在到位时冲过头
    straight_rpm: int = 30
    #: 原地转用多大转速。转弯不产生位移，不需要太快
    turn_rpm: int = 20
    #: 段长上限相对"段内超时"的折扣。0.6 是给爬行区、斜坡和 settle 留的余量：
    #: 位置环最后 400 计数会掉到 ≤15 RPM、最后 160 计数掉到 8 RPM（`:96-107`），
    #: 所以"计数/rpm"会明显低估真实耗时。
    timeout_safety: float = 0.6
    #: 主机侧的段看门狗：段下发后多久还没收到 0x84 的完成帧就认输（秒）。
    #: 必须**大于**固件的段内超时 `POSITION_TIMEOUT_S`（11.95 s）——
    #: 否则固件还没来得及判 TIMEOUT，主机先自己放弃了。
    #: 它防的是"完成帧丢了 / 固件压根没启动这一段"：没有它，执行器会在
    #: 那里**永远等下去**，而现象是"车不动、也不报错"。
    segment_watchdog_s: float = 15.0
    #: 连续多少段属于"可重试失败"（STALL/TIMEOUT）就放弃
    max_bad_streak: int = 3
    #: 连续多少段没有明显靠近目标就放弃。防的是"原地打转"这类活锁
    max_segments_without_progress: int = 6
    #: 算"靠近了"的最小距离改善（米）
    progress_epsilon_m: float = 0.02

    def __post_init__(self) -> None:
        for name in ("straight_rpm", "turn_rpm"):
            rpm = getattr(self, name)
            if not POSITION_RPM_MIN <= rpm <= POSITION_RPM_MAX:
                raise ValueError(
                    f"{name}={rpm} 越界：固件只接受 "
                    f"{POSITION_RPM_MIN}..{POSITION_RPM_MAX}"
                    f"（chassis_command_rx.v:109）")
        if self.min_segment_counts <= POSITION_MIN_COUNTS:
            raise ValueError(
                f"min_segment_counts={self.min_segment_counts} 必须大于 "
                f"{POSITION_MIN_COUNTS}，否则固件直接判 REJECTED"
                f"（chassis_position_controller.v:86）")
        if self.position_tolerance_m <= 0.0:
            raise ValueError("position_tolerance_m 必须为正")
        if self.heading_tolerance_rad <= 0.0:
            raise ValueError("heading_tolerance_rad 必须为正")
        if not 0.0 < self.timeout_safety < 1.0:
            raise ValueError("timeout_safety 必须在 (0, 1) 内")
        if self.segment_watchdog_s <= POSITION_TIMEOUT_S:
            raise ValueError(
                f"segment_watchdog_s={self.segment_watchdog_s} 必须大于固件的"
                f"段内超时 {POSITION_TIMEOUT_S}s —— 否则固件还没判 TIMEOUT，"
                f"主机就先放弃了，会把正常的长段误判成失败")


# ============================================================================
# 数据类型
# ============================================================================


@dataclass(frozen=True)
class TargetPoint:
    """目标点。`yaw` 为 None 表示"到点即可，不管朝向"。"""

    x: float
    y: float
    yaw: Optional[float] = None


@dataclass(frozen=True)
class Segment:
    """一段可以直接下发的 `0x05 POSITION`。"""

    mode: int
    max_rpm: int
    target_counts: int
    kind: str                   # 'straight' | 'turn'
    note: str = ""

    def position_kwargs(self) -> dict:
        """喂给 `pack_position` 的关键字参数。"""
        return {"mode": self.mode, "max_rpm": self.max_rpm,
                "target_counts": self.target_counts}

    def describe(self) -> str:
        name = {MODE_FORWARD: "前进", MODE_BACKWARD: "后退",
                MODE_ROTATE_LEFT: "原地左旋", MODE_ROTATE_RIGHT: "原地右旋"}
        return (f"{name.get(self.mode, self.mode)} {self.target_counts} 计数"
                f" @ {self.max_rpm} RPM")


class ActionKind(Enum):
    IDLE = "idle"               # 正在等当前段走完，什么都不用做
    SEND = "send"               # 立刻下发这个段
    ARRIVED = "arrived"         # 到达（含最终航向）
    FAILED = "failed"           # 放弃


@dataclass(frozen=True)
class Action:
    kind: ActionKind
    segment: Optional[Segment] = None
    reason: str = ""

    @property
    def terminal(self) -> bool:
        return self.kind in (ActionKind.ARRIVED, ActionKind.FAILED)


@dataclass(frozen=True)
class PlanResult:
    action: Action


# ============================================================================
# 规划器：位姿 + 目标点 → 下一段
# ============================================================================


class SegmentPlanner:
    """把"我在哪、要去哪"翻译成一段 `0x05 POSITION`。

    策略是**先转后走**：航向误差大就先原地转到大致朝向，然后走一段直线，
    走完再重新规划。因为每段结束都重新规划，所以小幅的航向残差会在下一段
    被纠正，不需要一次转准。
    """

    def __init__(self, geom: SkidSteerGeometry,
                 params: Optional[SegmentParams] = None) -> None:
        self.geom = geom
        self.p = params or SegmentParams()
        # 启动自检：航向容差必须**大到能指令得动**。
        # 转角计数 = θ·b_eff/2 / 周长 · CPR。容差太小 → 算出来的段小于
        # min_segment_counts → 永远转不动，规划器会卡在原地。
        counts_at_tol = geom.rotation_to_position_counts(self.p.heading_tolerance_rad)
        if counts_at_tol < self.p.min_segment_counts:
            raise ValueError(
                f"heading_tolerance_rad={self.p.heading_tolerance_rad} 太小："
                f"它对应的转角只有 {counts_at_tol:.1f} 计数，小于 "
                f"min_segment_counts={self.p.min_segment_counts}，"
                f"规划的转弯段会永远发不出去")
        # 到位容差必须大于最小段长，否则"没到位"时会算出太短的直线段
        counts_at_pos_tol = geom.distance_to_position_counts(self.p.position_tolerance_m)
        if counts_at_pos_tol <= self.p.min_segment_counts:
            raise ValueError(
                f"position_tolerance_m={self.p.position_tolerance_m} 太小："
                f"它对应的直线只有 {counts_at_pos_tol:.1f} 计数，"
                f"不大于 min_segment_counts={self.p.min_segment_counts}")

    # ---- 段长上限 ----

    def max_counts_for_rpm(self, rpm: int) -> int:
        """给定转速，一段最多能有多长（计数）。

        依据：段内超时 `POSITION_TIMEOUT_S`（`chassis_position_controller.v:20`）。
        超过它固件会判 `RESULT_TIMEOUT` 并且**停在那里**，所以这个上限必须
        严格遵守，不能"差不多就行"。
        """
        counts_per_s = rpm / 60.0 * self.geom.counts_per_rev
        usable_s = POSITION_TIMEOUT_S * self.p.timeout_safety
        return max(self.p.min_segment_counts, int(counts_per_s * usable_s))

    # ---- 主入口 ----

    def plan(self, pose: Pose2D, target: TargetPoint) -> Action:
        dx = target.x - pose.x
        dy = target.y - pose.y
        dist = math.hypot(dx, dy)

        if dist > self.p.position_tolerance_m:
            # 目标点还是有方向意义的；离得很近时 atan2 会抖，所以只在
            # 明显在外面时才用它算航向。
            err = wrap_angle(math.atan2(dy, dx) - pose.yaw)
            if abs(err) > self.p.heading_tolerance_rad:
                turn = self._turn_segment(err, "航向误差")
                if turn is not None:
                    return Action(ActionKind.SEND, turn)
                # 转不动（误差小到无法指令）→ 当作朝向够了，直接走
            straight = self._straight_segment(dist)
            if straight is None:
                # 距离已落在计数分辨率以内，算到位
                return Action(ActionKind.ARRIVED,
                              reason=f"距目标 {dist:.3f} m，已小于计数分辨率")
            return Action(ActionKind.SEND, straight)

        # 位置到了。若还指定了最终朝向，就地转过去。
        if target.yaw is not None:
            err = wrap_angle(target.yaw - pose.yaw)
            if abs(err) > self.p.heading_tolerance_rad:
                turn = self._turn_segment(err, "最终朝向")
                if turn is not None:
                    return Action(ActionKind.SEND, turn)
                return Action(ActionKind.ARRIVED,
                              reason=f"最终朝向残差 {err:.3f} rad，已小于计数分辨率")
        return Action(ActionKind.ARRIVED, reason="已到达目标")

    # ---- 两种段 ----

    def _turn_segment(self, err: float, why: str) -> Optional[Segment]:
        """`err` > 0 表示需要逆时针（yaw 增大）→ 原地左旋。"""
        counts = int(round(self.geom.rotation_to_position_counts(abs(err))))
        if counts < self.p.min_segment_counts:
            return None                       # 小到指令不动，交给调用方
        rpm = self.p.turn_rpm
        limit = self.max_counts_for_rpm(rpm)
        if counts > limit:
            why += "，超长已截断"
            counts = limit
        mode = MODE_ROTATE_LEFT if err > 0 else MODE_ROTATE_RIGHT
        return Segment(mode=mode, max_rpm=rpm, target_counts=counts,
                       kind="turn",
                       note=f"{why} {abs(err) * 180.0 / math.pi:.1f}°")

    def _straight_segment(self, dist: float) -> Optional[Segment]:
        counts = int(round(self.geom.distance_to_position_counts(dist)))
        if counts < self.p.min_segment_counts:
            return None
        rpm = self.p.straight_rpm
        limit = self.max_counts_for_rpm(rpm)
        note = ""
        if counts > limit:
            note = f"段长 {counts} 截断到上限 {limit}"
            counts = limit
        return Segment(mode=MODE_FORWARD, max_rpm=rpm, target_counts=counts,
                       kind="straight", note=note)


# ============================================================================
# 执行器：把段串起来，并决定何时放弃
# ============================================================================


class ExecState(Enum):
    IDLE = "idle"               # 没有目标
    READY = "ready"             # 有目标，可以规划下一段
    WAITING = "waiting"         # 段已下发，等 0x84 的 done
    ARRIVED = "arrived"
    FAILED = "failed"


class SegmentExecutor:
    """段级状态机。

    ⚠ 它**不认识串口**，只做决策。下发/保活/故障处理都在驱动里 ——
      这样分段策略的正确性可以在没有底盘、没有 ROS 的情况下被单测覆盖。
    """

    def __init__(self, planner: SegmentPlanner,
                 params: Optional[SegmentParams] = None) -> None:
        self.planner = planner
        self.p = params or planner.p
        self.state = ExecState.IDLE
        self.target: Optional[TargetPoint] = None
        self.current: Optional[Segment] = None
        self.fail_reason = ""
        # 统计（进诊断）
        #: 规划出来交给驱动去发的段数（含被限速退回后重新规划的，会重复计）
        self.segments_planned = 0
        #: **真的写进串口**的段数。由驱动在写成功后调 `note_sent()` 记账。
        #: ⚠ 断言"每段都成功"必须用它，不能用 `segments_planned`：
        #:   限速退让会让同一段被规划两次。
        self.segments_sent = 0
        self.segments_ok = 0
        self.send_failures = 0
        #: 当前段是什么时候下发的（单调时钟，用于看门狗）
        self.waiting_since: Optional[float] = None
        self.segments_retried = 0
        self.segments_aborted = 0
        self.travel_m = 0.0
        self.bad_streak = 0
        self.segments_without_progress = 0
        self.best_distance_m = float("inf")
        self.last_result: Optional[int] = None

    # ---- 目标 ----

    def set_goal(self, target: TargetPoint) -> None:
        self.target = target
        self.state = ExecState.READY
        self.fail_reason = ""
        self.current = None
        self.bad_streak = 0
        self.segments_without_progress = 0
        self.best_distance_m = float("inf")
        self.travel_m = 0.0
        self.waiting_since = None

    def cancel(self, reason: str = "被取消") -> None:
        self.state = ExecState.IDLE
        self.target = None
        self.current = None
        self.waiting_since = None
        self.fail_reason = ""

    @property
    def busy(self) -> bool:
        return self.state is ExecState.WAITING

    @property
    def active(self) -> bool:
        return self.state in (ExecState.READY, ExecState.WAITING)

    # ---- 拍 ----

    def poll(self, pose: Pose2D, now: Optional[float] = None) -> Action:
        """空闲时规划下一段；其他状态返回 IDLE 或既有的终态。

        `now` 是可选的单调时钟（秒）。给了它就会**检查段看门狗**：
        段下发后超过 `segment_watchdog_s` 还没等到 0x84 的完成帧，
        就判失败。不给就只看状态（纯逻辑单测里用）。
        """
        if self.state is ExecState.ARRIVED:
            return Action(ActionKind.ARRIVED, reason="已到达")
        if self.state is ExecState.FAILED:
            return Action(ActionKind.FAILED, reason=self.fail_reason)
        if self.state is ExecState.WAITING:
            # ★ 看门狗：不能"永远等一段"。
            #   0x84 的完成帧可能丢，或者固件压根没启动这一段（比如它还在
            #   busy 里，于是 start_valid && !busy 把命令静默丢了）。
            #   没有这条，症状是"车不动、不报错、目标永远不结束"。
            if now is not None and self.waiting_since is not None:
                waited = now - self.waiting_since
                if waited > self.p.segment_watchdog_s:
                    return self._fail_action(
                        f"段下发后 {waited:.1f}s 没收到 0x84 完成帧"
                        f"（看门狗 {self.p.segment_watchdog_s:.1f}s）"
                        + (f"：{self.current.describe()}"
                           if self.current is not None else ""))
            return Action(ActionKind.IDLE)
        if self.state is not ExecState.READY:
            return Action(ActionKind.IDLE)
        assert self.target is not None
        action = self.planner.plan(pose, self.target)
        if action.kind is ActionKind.SEND:
            self.current = action.segment
            self.state = ExecState.WAITING
            self.waiting_since = now
            self.segments_planned += 1
        elif action.kind is ActionKind.ARRIVED:
            self.state = ExecState.ARRIVED
        else:
            self._fail(action.reason)
            return Action(ActionKind.FAILED, reason=self.fail_reason)
        return action

    def note_sent(self) -> None:
        """驱动成功把当前段写进串口之后调这个。"""
        self.segments_sent += 1

    def on_send_deferred(self, reason: str = "命令限速，本拍发不出去") -> None:
        """本拍发不出去，但**这不是失败** —— 把段退回去，下一拍再来。

        ★ 必须和 `on_send_failed` 分开，这条是踩出来的：驱动的命令限速是
          50 ms（防 ACK 单槽撞车），而控制拍是 10 ms，所以"规划好的段被限速
          挡掉"每 5 拍就会发生一次。如果把它算进 `bad_streak`，**三次就放弃**
          —— 症状是"目标刚下发就报失败、车一动不动"，而且因为它依赖心跳和
          段的相位关系，看起来还是随机的。
        """
        self.current = None
        self.waiting_since = None
        self.state = ExecState.READY

    def on_send_failed(self, reason: str) -> Action:
        """段**真的**没发出去（串口写失败、编解码拒绝）。"""
        self.current = None
        self.waiting_since = None
        self.state = ExecState.READY
        self.send_failures += 1
        self.bad_streak += 1
        if self.bad_streak > self.p.max_bad_streak:
            return self._fail_action(f"连续 {self.bad_streak} 次下发失败：{reason}")
        return Action(ActionKind.IDLE, reason=f"下发失败，稍后重试：{reason}")

    def on_result(self, pose: Pose2D, result_code: int,
                  progress_counts: Optional[int] = None,
                  now: Optional[float] = None) -> Action:
        """收到 0x84 的完成帧（**上升沿**）时调用。返回下一个动作。

        ⚠ 调用方必须自己做上升沿检测：固件的 `done` 是电平，会在下一段 start
          之前一直被 0x84 报出来。照单全收会把上一段的完成帧当成新段的结果。
        """
        if self.state is not ExecState.WAITING:
            # 不是我们等的那一段（可能是别人直接往串口写的 POSITION）
            return Action(ActionKind.IDLE, reason="没有在等的段，忽略这次结果")

        seg = self.current
        self.current = None
        self.waiting_since = None
        self.last_result = int(result_code)
        if progress_counts:
            self.travel_m += self.planner.geom.position_counts_to_distance_m(
                abs(progress_counts))
        assert self.target is not None

        ok_result = 1                                        # PositionResult.OK
        if result_code == ok_result:
            self.segments_ok += 1
            self.bad_streak = 0
        elif int(result_code) in RETRYABLE_RESULTS:
            self.segments_retried += 1
            self.bad_streak += 1
            if self.bad_streak > self.p.max_bad_streak:
                name = "STALL" if int(result_code) == 3 else "TIMEOUT"
                return self._fail_action(
                    f"连续 {self.bad_streak} 段可重试失败（最后一段 {name}）"
                    + (f"：{seg.describe()}" if seg else ""))
            # 掉进这里说明要重试：用**当前位姿**重新规划，段会自然变短
            self.state = ExecState.READY
        else:
            # ABORTED / REJECTED / NONE：不重试
            self.segments_aborted += 1
            names = {0: "NONE", 2: "ABORTED", 5: "REJECTED"}
            name = names.get(int(result_code), f"未知({result_code})")
            return self._fail_action(
                f"段以 {name} 结束，不重试"
                + (f"：{seg.describe()}" if seg else ""))

        # 进度看门狗：防原地打转
        dist = math.hypot(self.target.x - pose.x, self.target.y - pose.y)
        if dist < self.best_distance_m - self.p.progress_epsilon_m:
            self.best_distance_m = dist
            self.segments_without_progress = 0
        else:
            self.segments_without_progress += 1
            if self.segments_without_progress > self.p.max_segments_without_progress:
                return self._fail_action(
                    f"连续 {self.segments_without_progress} 段没有靠近目标"
                    f"（最近 {self.best_distance_m:.3f} m，现在 {dist:.3f} m）")

        self.state = ExecState.READY
        return self.poll(pose, now)

    # ---- 失败 ----

    def _fail(self, reason: str) -> None:
        self.fail_reason = reason
        self.state = ExecState.FAILED
        self.current = None
        self.waiting_since = None
        self.target = None

    def _fail_action(self, reason: str) -> Action:
        self._fail(reason)
        return Action(ActionKind.FAILED, reason=reason)

    # ---- 诊断 ----

    def describe(self) -> str:
        tgt = (f"({self.target.x:.2f},{self.target.y:.2f})"
               if self.target is not None else "无")
        cur = self.current.describe() if self.current is not None else "-"
        line = (f"state={self.state.value} goal={tgt} cur={cur} "
                f"planned={self.segments_planned} sent={self.segments_sent} "
                f"ok={self.segments_ok} "
                f"retried={self.segments_retried} aborted={self.segments_aborted} "
                f"sendfail={self.send_failures} travel={self.travel_m:.3f}m")
        if self.fail_reason:
            line += f" fail={self.fail_reason}"
        return line
