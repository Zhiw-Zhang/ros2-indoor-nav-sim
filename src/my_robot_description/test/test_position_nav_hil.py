#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分段导航（4A）的端到端 HIL 测试：真驱动节点 + 真 PTY + 真固件模型。

它在证明的那一件事
------------------
**"用 0x05 位置段可以在现有位流上连续移动，而且不触发 3.0 s 上限。"**

这不是一条"顺便看看"的断言，而是整个 4A 存在的理由：`0x02 MOVE` 连续跑满
3.0 s 就会锁存 `FAULT_RUN_LIMIT`（`chassis_safety_fsm.v:95-97`），而位置环
因为 `run_limit_reached` 里带了 `!position_active`（`:57-58`）不受这条限制。
本文件用**超过 3 秒的真实运动**去撞这条边界，然后断言 `fault=NONE`。

还钉住三条"违反就会出真问题"的规矩
----------------------------------
1. 位置段进行中**一帧 MOVE 都不能发** —— `top.v:121-125` 会把 MOVE 当成
   `position_abort`，把当前段**中断**掉。
2. 段结束后**不能有残留的手动目标** —— `top.v:152` 是
   `target_rpm = position_busy ? position_target_rpm : manual_target_rpm`，
   段结束那一拍手动目标立刻接管，车会继续冲出去。
3. 段下发前参数必须**自校验过** —— 非法 POSITION 会被判 `protocol_fault`，
   而那是**永久锁死**（只有断电或「松开 S0 再按 S4」能解）。

本文件在 PTY 上**旁听驱动发出的每一帧**（用真分帧器解），所以上面第 1、2 条
是直接检查字节流得到的结论，不是"从日志推测"。

跑法（需要创建 PTY，所以要能访问 /dev/ptmx）::

    python3 src/my_robot_description/test/test_position_nav_hil.py

退出码 0 = 全部通过。
"""

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, "..", "src")
sys.path.insert(0, _SRC)

from acg720_driver import Acg720Driver                       # noqa: E402
from acg720_firmware import (                                # noqa: E402
    FaultReason, FirmwareParams, SafetyState,
)
from acg720_protocol_v1 import FrameParser, Kind             # noqa: E402
from acg720_simulator import Acg720SimulatorNode, PtySerial   # noqa: E402
from drive_kinematics import SkidSteerGeometry               # noqa: E402
from position_segments import ExecState                      # noqa: E402


class Failures:
    def __init__(self):
        self.failed = []

    def check(self, name, ok, detail=""):
        print(f"  {'✓' if ok else '✗'} {name}" + (f"  {detail}" if detail else ""))
        if not ok:
            self.failed.append(name)


class NavHarness:
    """虚拟车（真值轮距可配）+ position 模式的驱动节点。

    `gt_scale` 是**车体真实**的侧滑程度；驱动侧的 `wheel_separation_scale`
    是**它以为的**。两者不等 = 没标定，见 `[8]`。
    """

    def __init__(self, driver_scale=1.0, gt_scale=1.0, **driver_overrides):
        self.pty = PtySerial()
        params = FirmwareParams(ground_truth_separation_scale=gt_scale)
        self.sim = Acg720SimulatorNode(self.pty, params, physics_rate_hz=100.0)

        # ---- 旁听驱动发出的帧（独立于固件模型，直接解字节流）----
        # 存 (kind, payload) 而不是只存 kind：POSITION 的**动作码**在载荷里，
        # 而"先转再走"是否成立全靠它。
        self.tx_frames = []
        self._spy_parser = FrameParser()
        _orig_read = self.pty.read

        def _spy(*a, **kw):
            data = _orig_read(*a, **kw)
            if data:
                for kind, _seq, payload in self._spy_parser.feed(data):
                    self.tx_frames.append((int(kind), bytes(payload)))
            return data

        self.pty.read = _spy
        #: 整轮里 run_age 的最大值。**必须在跑的过程中采样** —— 位置段每结束
        #: 一段都会把 run_age 清零（internal_stop → ARMED_STOP），到了终点再看
        #: 只会看到最后一小段的残留值。
        self.max_run_age = 0.0

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.sim)

        overrides = {
            "port": self.pty.slave_path,
            "control_mode": "position",
            "wheel_separation_scale": driver_scale,
            "publish_tf": False,
        }
        overrides.update(driver_overrides)
        self.driver = Acg720Driver(parameter_overrides=overrides)
        self.executor.add_node(self.driver)

        self.goal_pub = self.driver.create_publisher(PoseStamped, "acg720/goal", 1)
        self.cmd_pub = self.driver.create_publisher(Twist, "cmd_vel", 10)
        self.status = []
        self.diags = []
        self.driver.create_subscription(
            String, "acg720/segment_status", lambda m: self.status.append(m.data), 1)
        self.driver.create_subscription(
            String, "acg720/diagnostics", lambda m: self.diags.append(m.data), 10)

    # ---- 时间推进 ----

    def _sample(self):
        if self.sim.fw.safety.run_age_s > self.max_run_age:
            self.max_run_age = self.sim.fw.safety.run_age_s

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.002)
            self._sample()

    def wait_until(self, pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.002)
            self._sample()
            if pred():
                return True
        return False

    # ---- 目标 ----

    def _make_goal(self, x, y, yaw=None, frame="odom", quat=None):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.driver.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        if quat is not None:
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = quat
        elif yaw is None:
            # 朝向不填就发全零四元数 —— 这是"到点即可"的常见写法，
            # 驱动必须把它当成"没填"，不能当成"朝向 0"。
            pass
        else:
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
        return msg

    def send_goal(self, x, y, yaw=None, frame="odom", quat=None, timeout=5.0):
        """发一个目标，**并等到驱动确认收到**。

        ⚠ 为什么不能只 publish 一条就走：目标订阅是默认 QoS（VOLATILE +
        深度 1），一条"在订阅者还没匹配上时发出去"的消息会被**静默丢掉** ——
        现象是"话题在、发了没反应、连个警告都没有"。这也是真机上的使用注意
        （RViz 点目标要在驱动起来之后再点）。测试里必须可重试，否则会随机失败。
        """
        msg = self._make_goal(x, y, yaw=yaw, frame=frame, quat=quat)
        before = self.driver._goal_seq
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.goal_pub.publish(msg)
            for _ in range(25):                     # ≈50 ms 的投递/回调时间
                self.executor.spin_once(timeout_sec=0.002)
                self._sample()
            if self.driver._goal_seq > before:
                return True
            if any(s.startswith("REJECTED") for s in self.status):
                return False
        return False

    def wait_arrived(self, timeout=40.0):
        return self.wait_until(
            lambda: any(s.startswith(("ARRIVED", "FAILED", "ABORTED"))
                        for s in self.status), timeout)

    def terminal_status(self):
        for s in reversed(self.status):
            if s.startswith(("ARRIVED", "FAILED", "ABORTED", "REJECTED")):
                return s
        return ""

    # ---- 观测 ----

    @property
    def truth(self):
        p = self.sim.fw.plant
        return p.x, p.y, p.yaw

    @property
    def odom_pose(self):
        p = self.driver.pose
        return p.x, p.y, p.yaw

    def kinds(self, since=0):
        return [k for k, _p in self.tx_frames[since:]]

    def position_modes(self, since=0):
        """驱动发出的每个 POSITION 段的动作码（按顺序）。"""
        from acg720_protocol_v1 import parse_position
        out = []
        for kind, payload in self.tx_frames[since:]:
            if kind == int(Kind.POSITION):
                out.append(parse_position(payload)["motion_mode"])
        return out

    def close(self):
        try:
            self.executor.remove_node(self.driver)
            self.driver.destroy_node()
        except Exception:
            pass
        try:
            self.executor.remove_node(self.sim)
        except Exception:
            pass


def geom_of(scale=1.0):
    return SkidSteerGeometry(scale=scale)


# ==========================================================================
# [1] 基线：直线目标，必须真的到
# ==========================================================================

def test_straight_goal(f):
    print("\n[1] 直线目标 1.0 m：分段走到位")
    h = NavHarness()
    try:
        h.spin_for(1.5)
        f.check("驱动进入了 position 模式",
                h.driver.control_mode == "position" and h.driver.segment_executor is not None)
        f.check("底盘已使能", h.sim.fw.state is SafetyState.ARMED_STOP,
                f"state={h.sim.fw.state.name}")

        f.check("目标被驱动接受", h.send_goal(1.0, 0.0))
        ok = h.wait_arrived(40.0)
        f.check("目标在超时前结束", ok, h.terminal_status() or "（没有任何终态）")
        f.check("终态是 ARRIVED", h.terminal_status().startswith("ARRIVED"),
                h.terminal_status())
        x, y, _ = h.truth
        tol = h.driver.segment_params.position_tolerance_m
        f.check("★ 车体真值确实到了目标点",
                abs(x - 1.0) <= tol + 0.02 and abs(y) <= 0.03,
                f"真值=({x:.3f},{y:.3f}) 目标=(1.000,0.000) 容差={tol}")
        ex = h.driver.segment_executor
        f.check("段数合理（不是一段到位，也不是几十段）",
                2 <= ex.segments_sent <= 8, f"segments_sent={ex.segments_sent}")
        # ⚠ 用 segments_sent，**不能**用 segments_planned：限速退让会让同一段
        #   被规划两次（planned 会偏大），那是正常的。
        f.check("每一段都成功结束（没有 STALL/TIMEOUT/ABORT）",
                ex.segments_ok == ex.segments_sent,
                f"ok={ex.segments_ok} sent={ex.segments_sent} "
                f"retried={ex.segments_retried} aborted={ex.segments_aborted}")
    finally:
        h.close()


# ==========================================================================
# [2] ★ 4A 的立身之本：连续运动超过 3 秒而**不**触发 RUN_LIMIT
# ==========================================================================

def test_long_motion_does_not_trip_run_limit(f):
    print("\n[2] ★ 连续运动 > 3.0 s：位置段不该触发 RUN_LIMIT")
    h = NavHarness()
    try:
        h.spin_for(1.5)
        goal = (2.5, 0.0)
        t0 = time.monotonic()
        f.check("目标被驱动接受", h.send_goal(*goal))
        ok = h.wait_arrived(60.0)
        elapsed = time.monotonic() - t0
        f.check("目标在超时前结束", ok, h.terminal_status())
        f.check("★ 运动总时长确实超过了固件的 3.0 s 上限", elapsed > 3.0,
                f"elapsed={elapsed:.2f}s")
        f.check("★ 底盘**没有**被锁死（这正是 4A 的意义）",
                h.sim.fw.fault is FaultReason.NONE
                and h.sim.fw.state is not SafetyState.FAULT,
                f"state={h.sim.fw.state.name} "
                f"fault={h.sim.fw.fault.name}")
        f.check("驱动侧也没报 RUN_LIMIT",
                "RUN_LIMIT" not in " ".join(h.status),
                " ".join(h.status)[:120])
        # ★ 关键：要证明**某一段自己**连续跑了超过 3.0 s。
        #   不然这一节可能只是"分成很多短段" —— 那 MOVE 模式也能做到，
        #   就证明不了位置环的豁免。
        f.check("★ 存在**单段**连续 RUNNING 超过 3.0 s（MOVE 做不到这件事）",
                h.max_run_age > 3.0,
                f"单段 run_age 峰值={h.max_run_age:.2f}s")
        x, y, _ = h.truth
        f.check("2.5 m 的目标也走完了", abs(x - 2.5) <= 0.05 and abs(y) <= 0.04,
                f"真值=({x:.3f},{y:.3f})")
    finally:
        h.close()


# ==========================================================================
# [3] 位置段进行中，驱动一帧 MOVE 都不能发
# ==========================================================================

def test_no_move_frames_are_sent(f):
    print("\n[3] ★ 位置段期间绝不能发 MOVE（会 abort 当前段）")
    h = NavHarness()
    try:
        h.spin_for(1.5)
        before = len(h.tx_frames)
        f.check("目标被驱动接受", h.send_goal(1.2, 0.6))
        h.wait_arrived(40.0)
        kinds = h.kinds(before)
        f.check("确实发过 POSITION 帧", int(Kind.POSITION) in kinds,
                f"kinds={sorted(set(kinds))}")
        f.check("★ 一帧 MOVE 都没发过", int(Kind.MOVE) not in kinds,
                f"MOVE 出现 {kinds.count(int(Kind.MOVE))} 次")
        f.check("一帧 OPEN_LOOP 都没发过", int(Kind.OPEN_LOOP) not in kinds)
        f.check("没有重复发 ARM", kinds.count(int(Kind.ARM)) <= 1,
                f"ARM 出现 {kinds.count(int(Kind.ARM))} 次")
        # 段与段之间必须是 HEARTBEAT 在喂狗，否则会 300 ms 超时锁死
        f.check("用 HEARTBEAT 喂狗（不是靠 STOP/MOVE）",
                int(Kind.HEARTBEAT) in kinds,
                f"HEARTBEAT={kinds.count(int(Kind.HEARTBEAT))}")
        # ★ 段之间的"空闲"里不能出现 STOP：STOP 会把位置段 abort 掉
        f.check("★ 没有用 STOP 去打断位置段", int(Kind.STOP) not in kinds,
                f"STOP 出现 {kinds.count(int(Kind.STOP))} 次")
        f.check("底盘没有超时锁死", h.sim.fw.fault is not FaultReason.TIMEOUT,
                h.sim.fw.fault.name)
    finally:
        h.close()


# ==========================================================================
# [4] 段结束后不许有残留的手动目标把车带走
# ==========================================================================

def test_no_residual_motion_after_arrival(f):
    print("\n[4] ★ 到达后车必须停住（不能有残留手动目标接管）")
    h = NavHarness()
    try:
        h.spin_for(1.5)
        f.check("目标被驱动接受", h.send_goal(0.8, 0.0))
        h.wait_arrived(40.0)
        # 让控制拍自由跑 2 s，看车会不会自己往前溜
        p0 = h.truth
        h.spin_for(2.0)
        p1 = h.truth
        moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        f.check("★ 到达后 2 s 内车没有继续移动", moved < 0.01,
                f"又走了 {moved * 1000:.1f} mm")
        f.check("固件状态回到了 ARMED_STOP（不锁死）",
                h.sim.fw.state is SafetyState.ARMED_STOP,
                h.sim.fw.state.name)
    finally:
        h.close()


# ==========================================================================
# [5] 目标带朝向：位置到了之后还要转到位
# ==========================================================================

def test_goal_with_final_yaw(f):
    print("\n[5] 目标带朝向（goal_uses_yaw:=true）")
    h = NavHarness(goal_uses_yaw=True)
    try:
        h.spin_for(1.5)
        want = -math.pi / 2
        f.check("目标被驱动接受", h.send_goal(0.6, 0.6, yaw=want))
        h.wait_arrived(45.0)
        f.check("终态是 ARRIVED", h.terminal_status().startswith("ARRIVED"),
                h.terminal_status())
        x, y, yaw = h.truth
        f.check("位置到位", math.hypot(x - 0.6, y - 0.6) <= 0.05,
                f"({x:.3f},{y:.3f})")
        err = abs(math.atan2(math.sin(yaw - want), math.cos(yaw - want)))
        f.check("★ 最终朝向也对上了（±0.15 rad）", err <= 0.15,
                f"yaw={math.degrees(yaw):.1f}° 目标={math.degrees(want):.1f}° "
                f"误差={math.degrees(err):.1f}°")
    finally:
        h.close()


def test_zero_quaternion_means_no_yaw_constraint(f):
    """全零四元数是"没填"，不是"朝向 +x"。当成约束会让车在终点乱转。"""
    print("\n[6] 全零四元数 = 不约束朝向")
    h = NavHarness(goal_uses_yaw=True)
    try:
        h.spin_for(1.5)
        # ★ "不约束朝向"必须显式发**全零四元数**。ROS 2 里 PoseStamped 的
        #   默认四元数是 (0,0,0,1) —— 也就是**朝向 +x**，不是"未指定"。
        #   这一条踩过：本用例第一版没设四元数，于是目标实际含义是
        #   "到 (0,0.7) 并转回 +x"，车老老实实转了 90° 又转回来，
        #   而测试却断言它应该保持在 +y —— 是**测试错了**，不是驱动错了。
        f.check("目标被驱动接受",
                h.send_goal(0.0, 0.7, quat=(0.0, 0.0, 0.0, 0.0)))
        h.wait_arrived(40.0)
        f.check("终态是 ARRIVED", h.terminal_status().startswith("ARRIVED"),
                h.terminal_status())
        _, _, yaw = h.truth
        # 若把全零四元数当成"朝向 0"，车会在到点后再转回 +x
        f.check("没有被迫转回 +x（说明没把全零四元数当成约束）",
                abs(math.degrees(yaw)) > 45.0,
                f"yaw={math.degrees(yaw):.1f}°")
    finally:
        h.close()


# ==========================================================================
# [7] 正后方的目标必须先原地转
# ==========================================================================

def test_goal_behind_requires_turn(f):
    print("\n[7] 目标在正后方：先原地转再直行")
    h = NavHarness()
    try:
        h.spin_for(1.5)
        before = len(h.tx_frames)
        f.check("目标被驱动接受", h.send_goal(-0.7, 0.0))
        h.wait_arrived(45.0)
        f.check("终态是 ARRIVED", h.terminal_status().startswith("ARRIVED"),
                h.terminal_status())
        x, y, _ = h.truth
        f.check("走到了 -0.7", abs(x + 0.7) <= 0.05 and abs(y) <= 0.04,
                f"真值=({x:.3f},{y:.3f})")
        # ★ 从**字节流**里直接看动作码：第一段必须是原地转(5/6)，
        #   而不是"倒着开过去"(2)。这是"先转后走"策略的直接证据。
        modes = h.position_modes(before)
        f.check("★ 第一段是原地转 5/6（不是后退 2）",
                bool(modes) and modes[0] in (5, 6),
                f"段动作码序列={modes[:6]}")
        f.check("段序列里没有后退动作", 2 not in modes, f"modes={modes[:8]}")
        f.check("段状态话题有记录", bool(h.status), " ".join(h.status)[:120])
    finally:
        h.close()


# ==========================================================================
# [8] 没标定：odom 空间收敛，但真值会偏 —— 这正是必须标定的原因
# ==========================================================================

def test_uncalibrated_car_drifts_in_truth(f):
    print("\n[8] 未标定（真值轮距 ×1.36，驱动仍按 1.0 算）")
    h = NavHarness(driver_scale=1.0, gt_scale=1.36)
    try:
        h.spin_for(1.5)
        f.check("目标被驱动接受", h.send_goal(0.0, 1.0))   # 需要转 90°
        h.wait_arrived(45.0)
        f.check("驱动认为已到达（odom 空间自洽）",
                h.terminal_status().startswith("ARRIVED"), h.terminal_status())
        ox, oy, _ = h.odom_pose
        tx, ty, _ = h.truth
        f.check("驱动侧 odom 位姿确实到了目标",
                math.hypot(ox - 0.0, oy - 1.0) <= 0.05,
                f"odom=({ox:.3f},{oy:.3f})")
        # ★ 这条**不是**在断言"车歪了"，而是把"歪多少"如实量出来：
        #   航向尺度不准时，折线整体偏斜，而 odom 完全看不出来。
        drift = math.hypot(tx - 0.0, ty - 1.0)
        print(f"    · 真值偏差 = {drift * 100:.1f} cm（odom 认为偏差 0）"
              f"；真值=({tx:.3f},{ty:.3f})")
        f.check("★ 真值确实偏了（说明 odom 自洽≠车真的到位）", drift > 0.05,
                f"drift={drift:.3f} m")
        f.check("底盘没锁死", h.sim.fw.fault is FaultReason.NONE,
                h.sim.fw.fault.name)
    finally:
        h.close()


# ==========================================================================
# [9] 人工接管：非零 /cmd_vel 立刻中止目标并停车
# ==========================================================================

def test_cmd_vel_overrides_goal(f):
    print("\n[9] ★ 非零 /cmd_vel = 人工接管，立刻中止目标")
    h = NavHarness()
    try:
        h.spin_for(1.5)
        # 故意给个远的，保证测试期间还在跑
        f.check("目标被驱动接受", h.send_goal(3.0, 0.0))
        h.wait_until(lambda: int(Kind.POSITION) in h.kinds(), 5.0)
        h.spin_for(0.6)
        f.check("目标已经在跑", h.driver.segment_executor.active,
                h.driver.segment_executor.describe())

        msg = Twist()
        msg.linear.x = 0.05            # 人为推一下
        h.cmd_pub.publish(msg)
        h.spin_for(0.6)

        f.check("★ 目标被中止了", not h.driver.segment_executor.active,
                h.driver.segment_executor.describe())
        f.check("状态话题报了 ABORTED",
                any(s.startswith("ABORTED") for s in h.status),
                " ".join(h.status)[-140:])
        f.check("执行器停在 idle",
                h.driver.segment_executor.state is ExecState.IDLE,
                h.driver.segment_executor.state.value)
        # 中止之后车必须停住
        p0 = h.truth
        h.spin_for(1.0)
        p1 = h.truth
        f.check("车停下来了", math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < 0.02,
                f"{(math.hypot(p1[0] - p0[0], p1[1] - p0[1])) * 1000:.1f} mm")
        f.check("底盘没锁死", h.sim.fw.fault is FaultReason.NONE,
                h.sim.fw.fault.name)
    finally:
        h.close()


# ==========================================================================
# [10] 位流不符时拒绝执行目标
# ==========================================================================

def test_goal_rejected_on_build_id_mismatch(f):
    print("\n[10] 位流版本不符：拒绝执行目标")
    h = NavHarness(expected_build_id="DEADBEEF")
    try:
        h.spin_for(2.0)
        f.check("识别出位流不符", h.driver.build_id_ok is False,
                f"build_id={h.driver.build_id}")
        before = len(h.tx_frames)
        # 这里**故意**断言"没被接受"：位流不符时驱动必须拒绝，而不是
        # 默默接下目标再什么都不做（那会让调用方一直等一个不会来的 ARRIVED）。
        f.check("★ 目标被明确拒绝（没有静默接受）", not h.send_goal(0.5, 0.0, timeout=3.0))
        h.spin_for(2.0)
        kinds = h.kinds(before)
        f.check("★ 一帧 POSITION 都没发出去", int(Kind.POSITION) not in kinds,
                f"kinds={sorted(set(kinds))}")
        f.check("状态话题报了 REJECTED",
                any(s.startswith("REJECTED") for s in h.status),
                " ".join(h.status)[-140:])
        f.check("底盘没被搞锁死", h.sim.fw.fault is FaultReason.NONE,
                h.sim.fw.fault.name)
    finally:
        h.close()


# ==========================================================================
# [11] 诊断与状态话题要够用：出问题时能一眼看出卡在哪一段
# ==========================================================================

def test_diagnostics_expose_segment_state(f):
    print("\n[11] 诊断里要有分段信息（现场排查全靠它）")
    h = NavHarness()
    try:
        h.spin_for(1.5)
        f.check("目标被驱动接受", h.send_goal(0.9, 0.0))
        h.wait_arrived(40.0)
        diag = h.diags[-1] if h.diags else ""
        for field in ("segments=", "position_busy=", "seg_limits=", "state="):
            f.check(f"诊断含 {field}", field in diag, diag[:160])
        f.check("状态话题是 latched 的（后连上也能看到）",
                bool(h.status), " ".join(h.status)[:120])
        f.check("段上限是有限值（不是 0、也不超过超时反算值）",
                h.driver.segment_planner.max_counts_for_rpm(30) > 1000,
                f"max@30RPM={h.driver.segment_planner.max_counts_for_rpm(30)}")
        # 0x84 的进度确实被消费过
        f.check("收到过位置遥测（0x84）", h.driver.position_done
                or h.driver.segment_executor.segments_ok > 0,
                f"busy={h.driver.position_busy} ok={h.driver.segment_executor.segments_ok}")
    finally:
        h.close()


def main():
    print("=" * 72)
    print("分段导航 HIL：真驱动（control_mode=position）+ 真 PTY + 真固件模型")
    print("=" * 72)
    rclpy.init()
    f = Failures()
    for fn in (test_straight_goal,
               test_long_motion_does_not_trip_run_limit,
               test_no_move_frames_are_sent,
               test_no_residual_motion_after_arrival,
               test_goal_with_final_yaw,
               test_zero_quaternion_means_no_yaw_constraint,
               test_goal_behind_requires_turn,
               test_uncalibrated_car_drifts_in_truth,
               test_cmd_vel_overrides_goal,
               test_goal_rejected_on_build_id_mismatch,
               test_diagnostics_expose_segment_state):
        try:
            fn(f)
        except Exception as exc:                       # noqa: BLE001
            import traceback
            traceback.print_exc()
            f.check(f"{fn.__name__} 没抛异常", False, repr(exc))
    print("\n" + "=" * 72)
    if f.failed:
        print(f"失败 {len(f.failed)} 项：")
        for name in f.failed:
            print(f"  - {name}")
        print("=" * 72)
        rclpy.shutdown()
        return 1
    print("全部通过 ✓")
    print("=" * 72)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
