#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端硬件在环（HIL）测试：**真驱动节点** + **真 PTY** + **真固件行为模型**。

分工
----
    单测文件（test_protocol_v1 / test_firmware_model / test_protocol_and_kinematics）
        钉住纯函数：编解码、固件行为、运动学。毫秒级、不需要 ROS。
    本文件
        钉住**集成行为**：把真正的 Acg720Driver 节点接到真正的 PTY 上，
        看 /cmd_vel 进去之后 /odom、TF、位姿、**故障上报**是不是对的。

所以这里验证的是"接线对不对、符号对不对、尺度对不对、坏消息传没传上来"，
这些正是真机上最难现场排查的东西。

跑法（需要创建 PTY，所以要能访问 /dev/ptmx）::

    python3 src/my_robot_description/test/test_hil_end_to_end.py

退出码 0 = 全部通过。
"""

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, "..", "src")
sys.path.insert(0, _SRC)

from acg720_driver import Acg720Driver                       # noqa: E402
from acg720_firmware import FirmwareParams, SafetyState, FaultReason  # noqa: E402
from acg720_protocol_v1 import (                             # noqa: E402
    Kind, SequenceTracker, encode_command, pack_move, pack_position,
)
from acg720_simulator import Acg720SimulatorNode, PtySerial   # noqa: E402
from drive_kinematics import SkidSteerGeometry                # noqa: E402


class Harness:
    """把虚拟车 + 驱动节点 + 一个外部旋进器装在同一条 spin 上。

    为什么要"同进程 + 单线程 executor + spin_once"而不是分进程：
    分进程的话，"命令发出去到车动起来"的时序要靠 sleep 凑，测试会 flaky；
    这里每个 tick 物理推进 10 ms、ROS 回调也跑一轮，时序是确定的。
    """

    #: 虚拟车的真值有效轮距倍率。代表"真实车体"的侧滑程度。
    GT_SCALE = 1.36

    def __init__(self, driver_scale=1.0, publish_tf=True, **sim_params):
        self.pty = PtySerial()
        params = FirmwareParams(ground_truth_separation_scale=self.GT_SCALE,
                                **sim_params)
        self.sim = Acg720SimulatorNode(self.pty, params, physics_rate_hz=100.0)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.sim)
        self.driver = None
        self.cmd_pub = None
        self.odoms = []
        self.odom_times = []
        self.diags = []
        self.latched = []
        self.attach_driver(driver_scale=driver_scale, publish_tf=publish_tf)

    def attach_driver(self, driver_scale=1.0, publish_tf=True, **overrides):
        """（重新）挂一个驱动节点到同一条 PTY 上。

        用于"驱动重启后 SEQ 必须重新对齐"这一条：新的驱动 SEQ 从 0 开始，
        而固件还记着上一次的序号 —— 不对齐的话新命令会被判重复而**静默丢弃**。
        """
        # ★ 必须在**建节点时**传参数：驱动的 __init__ 末尾就会打开串口并按
        #   参数算运动学，建完再 set_parameters 已经太晚。
        params = {
            "port": self.pty.slave_path,
            "wheel_separation_scale": driver_scale,
            "publish_tf": publish_tf,
            # 放宽驱动侧限幅，让测试能单独考察固件侧的拒绝/投影逻辑
            "max_linear_vel": 2.0,
            "max_angular_vel": 6.0,
        }
        params.update(overrides)
        return self.attach_existing(Acg720Driver(parameter_overrides=params))

    def attach_existing(self, driver):
        """把一个已经建好的驱动节点接上 executor 与各订阅。"""
        self.driver = driver
        self.executor.add_node(driver)
        self.cmd_pub = driver.create_publisher(Twist, "cmd_vel", 10)
        def _on_odom(msg, _h=self):
            _h.odoms.append(msg)
            _h.odom_times.append(time.monotonic())
        driver.create_subscription(Odometry, "odom", _on_odom, 20)
        driver.create_subscription(String, "acg720/diagnostics",
                                   lambda m: self.diags.append(m.data), 10)
        driver.create_subscription(String, "acg720/diagnostics_latched",
                                   lambda m: self.latched.append(m.data), 10)
        return driver

    def detach_driver(self):
        if self.driver is None:
            return
        try:
            self.executor.remove_node(self.driver)
            self.driver.destroy_node()
        except Exception:
            pass
        self.driver = None

    # ---------------- 驱动 ----------------

    def spin_for(self, seconds, cmd=None, rate=0.02, publish_cmd=True):
        """跑 seconds 秒。cmd=(v, ω) 时按 rate 持续发 /cmd_vel。"""
        end = time.monotonic() + seconds
        last_cmd = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if cmd is not None and publish_cmd and now - last_cmd >= rate:
                last_cmd = now
                self.cmd_pub.publish(_twist(*cmd))
            self.executor.spin_once(timeout_sec=0.002)

    def latest_odom(self):
        return self.odoms[-1] if self.odoms else None

    def latest_diag(self):
        return self.diags[-1] if self.diags else ""

    def reset_pose(self):
        """把驱动侧积分位姿**和虚拟车真值位姿**一起清零。

        ⚠ 必须两边都清。只清驱动侧的话，下一步测试比的是"里程计增量 vs 累计真值"，
        比值会莫名其妙地偏小（本项目就踩过）。这种污染在单独跑某一项时不会出现，
        最容易被误判成"物理模型不对"。

        ⚠⚠ 只清位姿**不够**：`run_age` 是"连续 RUNNING 时长"，它只在离开 RUNNING
          时清零。共用同一个 harness 的多个用例如果一路 RUNNING 跑下来，
          累计超过 3.0 s 就会锁死底盘，后面的用例全都会莫名其妙地"车不动"。
          （本文件第一版就是这么错的：`[3]` 的第二次旋转只转了 23°，`[4]` 直接是 0。）
          所以共用 harness 的用例要用 `stop_and_reset()`。
        """
        self.driver.pose.x = 0.0
        self.driver.pose.y = 0.0
        self.driver.pose.yaw = 0.0
        self.sim.fw.plant.x = 0.0
        self.sim.fw.plant.y = 0.0
        self.sim.fw.plant.yaw = 0.0
        self.odoms.clear()
        self.odom_times.clear()

    def stop_and_reset(self, settle=1.0):
        """先停车、等 `run_age` 归零，再清位姿。共用 harness 的用例必须走这条。"""
        end = time.monotonic() + settle
        last_cmd = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last_cmd >= 0.02:
                last_cmd = now
                self.cmd_pub.publish(_twist(0.0, 0.0))
            self.executor.spin_once(timeout_sec=0.002)
            if (self.driver.fw_state == SafetyState.ARMED_STOP
                    and self.sim.fw.safety.run_age_s == 0.0):
                break
        self.reset_pose()

    def rates_over(self, seconds, cmd, skip=0.6, rate=0.02):
        """同一时间窗内同时采样里程计与真值，返回 (w_odom, w_true)。

        ⚠ 两条信号必须**同窗**测量。分别测再相除是不对的：电机是一阶响应，
        两次测量的起步瞬态不同，比值会带上与尺度无关的误差。
        """
        t0 = time.monotonic()
        samples = []
        end = t0 + seconds
        last_cmd = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last_cmd >= rate:
                last_cmd = now
                self.cmd_pub.publish(_twist(*cmd))
            self.executor.spin_once(timeout_sec=0.002)
            o = self.latest_odom()
            if o is not None:
                samples.append((now - t0,
                                _yaw_from_quat(o.pose.pose.orientation),
                                self.sim.fw.plant.yaw))
        late = [s for s in samples if s[0] >= skip]
        if len(late) < 3:
            return float("nan"), float("nan")
        dt = late[-1][0] - late[0][0]
        if dt <= 0:
            return float("nan"), float("nan")
        return (_unwrap_total([s[1] for s in late]) / dt,
                _unwrap_total([s[2] for s in late]) / dt)

    def close(self):
        self.detach_driver()
        try:
            self.sim.destroy_node()
        except Exception:
            pass
        self.pty.close()


# ==========================================================================
# 断言工具
# ==========================================================================

class Failures:
    def __init__(self):
        self.items = []

    def check(self, name, ok, detail=""):
        mark = "✓" if ok else "✗"
        print(f"  {mark} {name}" + (f"  {detail}" if detail else ""))
        if not ok:
            self.items.append(name)

    def approx(self, name, got, want, tol, unit=""):
        ok = abs(got - want) <= tol
        self.check(name, ok,
                   f"got={got:.4f}{unit} want={want:.4f}±{tol}{unit}")
        return ok


# ==========================================================================
# 用例
# ==========================================================================

def test_link_come_up(f: Failures, h: Harness) -> None:
    print("\n[1] 链路与握手")
    h.spin_for(0.6)
    f.check("串口已打开", h.driver.serial_connected, str(h.driver.port))
    f.check("收到了遥测帧（0x81）", h.driver.telemetry_count > 3,
            f"telemetry={h.driver.telemetry_count}")
    f.check("收到了 ACK（0x80）", h.driver.ack_count >= 1,
            f"acks={h.driver.ack_count}")
    f.check("位流版本号校验通过", h.driver.build_id_ok is True,
            f"build_id={'N/A' if h.driver.build_id is None else hex(h.driver.build_id)}")
    f.check("SEQ 已按固件上报对齐", h.driver.seq_synced,
            f"下一条 SEQ={h.driver.seq.sequence}")
    f.check("自动 ARM 后进入 ARMED_STOP",
            h.driver.fw_state == SafetyState.ARMED_STOP,
            f"state={h.driver.fw_state}")
    f.check("没有 CRC 错误", h.driver._parser.crc_errors == 0)
    f.check("没有丢字节", h.driver._parser.discarded_bytes == 0)


def test_move_and_odom(f: Failures, h: Harness) -> None:
    print("\n[2] 直行：/cmd_vel → MOVE → /odom + TF")
    h.stop_and_reset()
    h.spin_for(1.2, cmd=(0.10, 0.0))
    o = h.latest_odom()
    f.check("收到了 /odom", o is not None)
    if o is None:
        return
    f.check("odom frame_id = odom", o.header.frame_id == "odom")
    f.check("odom child_frame_id = base_link", o.child_frame_id == "base_link")
    f.check("前进使 x 增大", o.pose.pose.position.x > 0.03,
            f"x={o.pose.pose.position.x:.4f}")
    f.check("y 基本不变（没有侧偏）", abs(o.pose.pose.position.y) < 0.02,
            f"y={o.pose.pose.position.y:.4f}")
    f.check("驱动进入了 RUNNING 或已到上限",
            h.driver.fw_state in (SafetyState.RUNNING, SafetyState.FAULT),
            f"state={h.driver.fw_state}")
    f.check("确实发出过运动命令", h.driver.motion_cmd_count > 0,
            f"motion_cmds={h.driver.motion_cmd_count}")
    lm = h.driver.last_move
    f.check("用的是直线动作（mode 1）", lm is not None and lm.motion_mode == 1,
            f"mode={None if lm is None else lm.motion_mode}")


def test_rotation_direction(f: Failures, h: Harness) -> None:
    print("\n[3] 原地转：左右符号")
    for sign, want in ((+1.0, True), (-1.0, False)):
        h.stop_and_reset()
        h.spin_for(1.0, cmd=(0.0, sign * 0.8))
        o = h.latest_odom()
        if o is None:
            f.check(f"ω={sign:+.1f} 有 /odom", False)
            continue
        yaw = _yaw_from_quat(o.pose.pose.orientation)
        f.check(f"ω={sign:+.1f} → 车体 {'左' if sign > 0 else '右'}转",
                (yaw > 0.05) if want else (yaw < -0.05),
                f"yaw={math.degrees(yaw):+.1f}deg")


def test_odom_scale_uncorrected(f: Failures, h: Harness) -> None:
    print("\n[4] 未标定的里程计：odom 会比真值转得多")
    h.stop_and_reset()
    w_odom, w_true = h.rates_over(1.6, (0.0, 0.9))
    if not (w_odom == w_odom and w_true == w_true and abs(w_true) > 1e-6):
        f.check("采到足够的转角样本", False, f"w_odom={w_odom} w_true={w_true}")
        return
    ratio = w_odom / w_true
    # 虚拟车的真值有效轮距 = 几何 × 1.36，所以 odom 报的角速度是真值的 1.36 倍
    f.approx("odom/真值 ≈ 1.36（未标定）", ratio, h.GT_SCALE, 0.25)


def test_odom_scale_corrected(f: Failures) -> None:
    print("\n[5] 标定后：把 wheel_separation_scale 打到 1.36 就该对上")
    h = Harness(driver_scale=Harness.GT_SCALE)
    try:
        h.spin_for(0.6)
        h.reset_pose()
        w_odom, w_true = h.rates_over(1.6, (0.0, 0.9))
        if not (w_odom == w_odom and w_true == w_true and abs(w_true) > 1e-6):
            f.check("采到足够的转角样本", False, f"w_odom={w_odom} w_true={w_true}")
            return
        f.approx("odom/真值 ≈ 1.0（已标定）", w_odom / w_true, 1.0, 0.12)
    finally:
        h.close()


def test_cmd_timeout_stops(f: Failures, h: Harness) -> None:
    print("\n[6] 停发 /cmd_vel 后必须自动停车")
    h.stop_and_reset()
    h.spin_for(0.8, cmd=(0.1, 0.0))
    h.spin_for(0.8)                       # 超过 cmd_vel_timeout，且不再发
    o1 = h.latest_odom()
    h.spin_for(0.6)
    o2 = h.latest_odom()
    if o1 is None or o2 is None:
        f.check("超时后仍能读到 /odom", False)
        return
    d = abs(o2.pose.pose.position.x - o1.pose.pose.position.x)
    f.check("超时后位置不再变化", d < 0.01, f"Δx={d:.4f}m")
    f.check("驱动侧记下了超时", h.driver.cmd_timed_out)


def test_no_heartbeat_during_motion(f: Failures, h: Harness) -> None:
    """★ 回归：**运动期间绝不能发 HEARTBEAT**。

    `top.v:238-281` 里任何非 MOVE 的有效命令都会把手动目标清零 ——
    如果驱动按老设计"100 ms 心跳喂狗"，车会在运动中被反复刹停，
    而状态机还停在 RUNNING（日志上看不出任何异常）。
    所以这里看的是"速度有没有掉下来"，不是"状态对不对"。
    """
    print("\n[7] 运动期间不得插入 HEARTBEAT（否则车会走走停停）")
    h.stop_and_reset()
    h.spin_for(1.8, cmd=(0.10, 0.0))
    lasts = [d for d in h.diags if "state=RUNNING" in d]
    f.check("测到 RUNNING 状态", bool(lasts), f"{len(lasts)} 条诊断")

    # ⚠ 时间戳必须用**收到 /odom 的时刻**（订阅回调里记的），
    #   不能用"读列表的时刻" —— 那样所有样本的时间几乎相同，速度会算出天文数字。
    n = min(len(h.odoms), len(h.odom_times))
    # 跳过起步的斜坡段（一阶响应 ~0.12 s + 位置环限幅），只看稳态
    xs = [(h.odom_times[i], h.odoms[i].pose.pose.position.x)
          for i in range(n)][12:45]
    if len(xs) < 10:
        f.check("采到足够的 /odom 样本", False, f"{len(xs)} 个")
        return
    span = xs[-1][0] - xs[0][0]
    dist = xs[-1][1] - xs[0][1]
    v_mean = dist / span if span > 0 else 0.0
    f.check("平均前进速度接近命令值 0.10 m/s", abs(v_mean - 0.10) < 0.03,
            f"v={v_mean:.4f} m/s")
    # 逐段增量里不能出现"停住"的片段 —— 那正是 HEARTBEAT 插进来的特征
    worst = 0.0
    for (t0, x0), (t1, x1) in zip(xs, xs[1:]):
        dt = t1 - t0
        if dt > 0.005:
            worst = max(worst, 0.10 - (x1 - x0) / dt)
    f.check("没有任何一段掉到接近静止", worst < 0.06,
            f"最差段比命令慢 {worst:.4f} m/s")


def test_run_limit_is_observed_and_reported(f: Failures) -> None:
    """★ 核心：连续运动 3.0 s 后固件锁死，**驱动必须把这件事报出来并停止发命令**。

    这是当前固件的已知限制（`chassis_safety_fsm.v:11,57-58,72-77`），
    不是主机 bug。真机上它意味着"连续导航今天做不到"——
    所以这个测试的价值是"让这个结论在 CI 里一直成立"，
    而不是"验证一个正常行为"。
    """
    print("\n[8] 3.0 s 连续运动上限：必须被观测到并如实上报")
    h = Harness()
    try:
        h.spin_for(0.6)
        f.check("上限常量的默认值就是固件真值 3.0 s",
                abs(h.sim.fw.p.run_limit_s - 3.0) < 1e-9,
                f"run_limit_s={h.sim.fw.p.run_limit_s}")
        h.reset_pose()
        h.spin_for(4.5, cmd=(0.10, 0.0))

        faults = [d for d in h.diags if "fault=RUN_LIMIT" in d]
        f.check("诊断里报出了 RUN_LIMIT", bool(faults),
                f"最后一条：{h.latest_diag()[:120]}")
        f.check("锁存诊断里写清了'这是固件限制'",
                any("连续运动超过 3.0 s" in s for s in h.latched),
                f"latched={h.latched}")
        f.check("驱动停止了运动命令", h.driver.motion_blocked,
                f"block_reason={h.driver.block_reason}")
        f.check("底盘确实停在 FAULT",
                h.sim.fw.state == SafetyState.FAULT
                and h.sim.fw.fault == FaultReason.RUN_LIMIT,
                f"state={h.sim.fw.state.name} fault={h.sim.fw.fault.name}")
        f.check("没有触发 tx_overflow（命令速率是安全的）",
                h.sim.fw._tx_overflow_count == 0,
                f"tx_overflow={h.sim.fw._tx_overflow_count}")

        before = h.driver.motion_cmd_count
        h.spin_for(0.5, cmd=(0.10, 0.0))
        f.check("锁死之后不再发运动命令",
                h.driver.motion_cmd_count == before,
                f"{before} → {h.driver.motion_cmd_count}")
    finally:
        h.close()


def test_overspeed_is_refused_locally(f: Failures) -> None:
    """超过 200 RPM 的目标**在主机上就被拒**，不会把车搞锁死。

    旧设计是"发出去、等固件拒绝"；可是固件对越界目标的反应是
    `protocol_fault` → **永久锁死**，不是"拒绝并继续"。所以必须在编码期拦住。
    """
    print("\n[9] 超限目标必须在主机侧拦下（发出去会锁死底盘）")
    h = Harness()
    try:
        h.spin_for(0.6)
        h.spin_for(0.6, cmd=(5.0, 0.0))       # 远超 200 RPM
        f.check("底盘没有被搞锁死",
                h.sim.fw.state != SafetyState.FAULT,
                f"state={h.sim.fw.state.name} fault={h.sim.fw.fault.name}")
        f.check("计到了'被投影/拒绝'的次数", h.driver.projection_count > 0,
                f"projected={h.driver.projection_count}")
        f.check("固件侧没有收到非法帧",
                h.sim.fw.rx.crc_errors == 0 and h.sim.fw.rx.frame_errors == 0
                and h.sim.fw._protocol_fault_count == 0,
                f"protocol_fault={h.sim.fw._protocol_fault_count}")
    finally:
        h.close()


def test_seq_resync_after_driver_restart(f: Failures) -> None:
    """★ 驱动重启后 SEQ 必须重新对齐，否则命令会被**静默丢弃**。

    现象极难查：话题有、驱动有、日志没有报错，车就是不动
    （固件把它判成重复帧，只加 `duplicate_count`，不回 ACK）。
    """
    print("\n[10] 驱动重启：SEQ 重新对齐")
    h = Harness()
    try:
        h.spin_for(0.6)
        h.spin_for(0.5, cmd=(0.1, 0.0))
        seq_before = h.sim.fw.rx.last_seq
        dups_before = h.sim.fw.rx.duplicates

        h.detach_driver()
        h.spin_for(0.3)
        d = h.attach_driver()
        f.check("新驱动的 SEQ 从头开始", d.seq.sequence == 0, f"seq={d.seq.sequence}")

        h.reset_pose()
        h.spin_for(1.0, cmd=(0.1, 0.0))
        f.check("固件的 last_seq 被继续推进了",
                h.sim.fw.rx.last_seq != seq_before,
                f"{seq_before} → {h.sim.fw.rx.last_seq}")
        f.check("没有任何一帧被判成重复（说明对齐成功）",
                h.sim.fw.rx.duplicates == dups_before,
                f"duplicates {dups_before} → {h.sim.fw.rx.duplicates}")
        f.check("新驱动确实让车动起来了",
                h.latest_odom() is not None
                and h.latest_odom().pose.pose.position.x > 0.02,
                f"x={0.0 if h.latest_odom() is None else h.latest_odom().pose.pose.position.x:.4f}")
    finally:
        h.close()


def test_short_glitch_is_survivable(f: Failures) -> None:
    """★ 短掉线（< 300 ms）要能自己救回来 —— 这就是把重连周期压到 0.1 s 的理由。

    固件的判据是"300 ms 内必须有有效命令"（`chassis_safety_fsm.v:10,56`），
    所以重连周期是**安全参数**，不是随便调的工程细节：默认 1.0 s 的话，
    任何一次串口抖动都会把底盘锁死，而且锁死之后重连成功也没用。
    """
    print("\n[11] 短掉线：重连要抢在 300 ms 之内，否则底盘锁死")
    h = Harness()
    try:
        h.spin_for(0.6)
        opens_before = h.driver._open_count
        h.driver._close_serial()
        h.spin_for(1.0)

        f.check("自动重连过（串口重新打开）", h.driver._open_count > opens_before,
                f"open_count {opens_before} → {h.driver._open_count}")
        f.check("底盘没有被锁死（抢在 300 ms 内）",
                h.sim.fw.state != SafetyState.FAULT,
                f"state={h.sim.fw.state.name} fault={h.sim.fw.fault.name}")
        f.check("重连后 SEQ 重新对齐过", h.driver.seq_synced)
        f.check("重连后重新 ARM 过", h.driver.arm_sent)

        h.stop_and_reset()
        h.spin_for(1.0, cmd=(0.1, 0.0))
        o = h.latest_odom()
        f.check("重连后还能走", o is not None and o.pose.pose.position.x > 0.02,
                f"x={0.0 if o is None else o.pose.pose.position.x:.4f}")
    finally:
        h.close()


def test_long_glitch_latches_and_recovers_after_reset(f: Failures) -> None:
    """★ 长掉线（> 300 ms）会把底盘**锁死**，重连成功也救不回来。

    这不是本节点的缺陷，是固件的安全策略。所以驱动必须：
      · 如实报出来（而不是继续发一堆没用的帧假装正常）
      · 停止发运动命令
      · 外部复位之后能自动恢复（重新对齐、重新 ARM、继续能走）
    """
    print("\n[12] 长掉线：锁死必须被报出来，复位后要能自动恢复")
    h = Harness()
    try:
        h.spin_for(0.6)
        # 把重连周期调慢，模拟"断开很久"（真机上就是串口线掉了）
        h.driver.set_parameters([
            rclpy.parameter.Parameter("reconnect_period",
                                      value=2.0)])
        h.driver._close_serial()
        h.spin_for(1.2)                       # 断开 > 300 ms

        f.check("底盘被锁死（固件的 300 ms 命令超时）",
                h.sim.fw.state == SafetyState.FAULT
                and h.sim.fw.fault == FaultReason.TIMEOUT,
                f"state={h.sim.fw.state.name} fault={h.sim.fw.fault.name}")

        h.spin_for(1.5)                       # 等重连完成并读到锁死状态
        f.check("驱动把锁死报了出来", h.driver.motion_blocked,
                f"block_reason={h.driver.block_reason}")
        f.check("锁存诊断里说明了原因",
                any("TIMEOUT" in s for s in h.latched), f"{h.latched}")

        # ---- 模拟外部复位（断电 / 松开 S0 再按 S4）----
        h.sim.fw.reset()
        h.spin_for(1.5)
        f.check("复位后驱动解除了封锁", not h.driver.motion_blocked,
                f"block_reason={h.driver.block_reason}")
        f.check("复位后重新 ARM 了", h.driver.arm_sent)

        h.stop_and_reset()
        h.spin_for(1.0, cmd=(0.1, 0.0))
        o = h.latest_odom()
        f.check("复位后还能走", o is not None and o.pose.pose.position.x > 0.02,
                f"x={0.0 if o is None else o.pose.pose.position.x:.4f}")
    finally:
        h.close()


def test_build_id_mismatch_refuses_motion(f: Failures) -> None:
    """位流版本不符 → **拒绝任何运动命令**，而且要说清楚。

    交接包的 `position_control.py:110-113` 就是这么做的：宁可不动，
    也不能拿错误位流按错误的口径控制真车。
    """
    print("\n[13] 位流版本不符：拒绝运动")
    h = Harness()
    try:
        # 直接改参数不方便（驱动已建），所以 destroy/重建 一个期望错误 build_id 的
        h.detach_driver()
        d = h.attach_driver(expected_build_id="DEADBEEF")
        h.spin_for(1.0, cmd=(0.1, 0.0))
        f.check("识别出位流不符", d.build_id_ok is False,
                f"build_id={'N/A' if d.build_id is None else hex(d.build_id)}")
        f.check("拒绝发送运动命令", d.motion_blocked and d.motion_cmd_count == 0,
                f"motion_cmds={d.motion_cmd_count}")
        f.check("底盘没有被误锁死", h.sim.fw.state != SafetyState.FAULT,
                f"state={h.sim.fw.state.name}")
        f.check("锁存诊断里写明了位流不符",
                any("MISMATCH" in s for s in h.latched), f"{h.latched}")
    finally:
        h.close()


def test_position_telemetry_surfaces(f: Failures) -> None:
    """0x05 POSITION 的进度（0x84）必须能被驱动报出来，且能报出最终结果。

    当前驱动不主动发 POSITION（那是独立的一期工作），但**解码链路**要通 ——
    否则等真去做分段导航时会发现"进度根本收不到"。

    ⚠ 位置动作期间**同样需要喂狗**：固件的 300 ms 命令超时对 RUNNING 一视同仁
      （`chassis_safety_fsm.v:56,116-119`），厂商的 `position_control.py` 就是
      每 100 ms 发一条心跳。所以这里由测试自己发心跳。
    """
    print("\n[14] 位置遥测 0x84 的解码链路")
    h = Harness()
    try:
        h.spin_for(0.6)
        trace = [f"start:{h.sim.fw.state.name}/seq{h.sim.fw.rx.last_seq}"
                 f"/cmds{h.driver.cmd_count}"]
        # 换一个 auto_arm=False 的驱动：它一条命令都不发，
        # 于是 SEQ 的所有权完全交给我们，不会和驱动抢序号。
        h.detach_driver()
        trace.append(f"detached:{h.sim.fw.state.name}/seq{h.sim.fw.rx.last_seq}"
                     f"/dups{h.sim.fw.rx.duplicates}")
        # ⚠ 拆卸之后必须**先转一会儿**再挂新的：旧驱动退出时会发一条 DISARM，
        #   而 PTY 从机一关闭，master 读队列里还没被取走的字节可能就没了 ——
        #   于是固件留在 ARMED_STOP、300 ms 后被判超时而锁死，测试就变成在测
        #   "锁死之后的行为"。这是 PTY 的特性，不是驱动的问题（真串口上
        #   我们本来就先发 DISARM 再 sleep 再关）。
        h.spin_for(0.2)
        trace.append(f"after_detach_spin:{h.sim.fw.state.name}/seq{h.sim.fw.rx.last_seq}")
        h.attach_driver(auto_arm=False)
        h.spin_for(0.3)
        trace.append(f"after_attach_spin:{h.sim.fw.state.name}/seq{h.sim.fw.rx.last_seq}")

        seq = SequenceTracker(sequence=h.sim.fw.rx.last_seq + 1)
        arm_seq = seq.take()
        h.pty.write_to_slave(encode_command(Kind.ARM, arm_seq))
        # ★ 只等 0.15 s，**不能等 0.3 s**：固件的命令超时正好是 300 ms，
        #   等满 300 ms 就等于让刚使能的底盘自己超时锁死。
        #   这个 0.3 把测试变成了"和超时赛跑"，会随机失败 —— 本项目真的被它坑过。
        h.spin_for(0.15)
        f.check("直接下发的 ARM 被接受（状态机进 ARMED_STOP）",
                h.sim.fw.state == SafetyState.ARMED_STOP,
                f"state={h.sim.fw.state.name} fault={h.sim.fw.fault.name} "
                f"fw_last_seq={h.sim.fw.rx.last_seq} arm_seq={arm_seq} "
                f"dups={h.sim.fw.rx.duplicates} trace={' | '.join(trace)}")

        h.pty.write_to_slave(encode_command(
            Kind.POSITION, seq.take(), pack_position(1, 25, 1000)))

        saw_busy = False
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            # 位置动作期间照样要喂狗（300 ms 超时对 RUNNING 一样有效）
            h.pty.write_to_slave(encode_command(Kind.HEARTBEAT, seq.take()))
            h.spin_for(0.1)
            saw_busy = saw_busy or h.driver.position_busy
            if h.driver.position_result is not None:
                break

        f.check("驱动收到了 POSITION 遥测并报出 busy", saw_busy,
                f"busy={h.driver.position_busy}")
        f.check("驱动报出了最终结果 OK", h.driver.position_result == 1,
                f"result={h.driver.position_result}")
        f.check("位置动作确实走完了", h.sim.fw.position.result_code.name == 'OK',
                f"progress={h.sim.fw.position.progress_counts} "
                f"result={h.sim.fw.position.result_code.name}")
        f.check("期间没有把底盘搞锁死",
                h.sim.fw.state != SafetyState.FAULT,
                f"state={h.sim.fw.state.name} fault={h.sim.fw.fault.name}")
    finally:
        h.close()


# ==========================================================================
# 小工具
# ==========================================================================

def _twist(v, w):
    m = Twist()
    m.linear.x = float(v)
    m.angular.z = float(w)
    return m


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _unwrap_total(yaws):
    """把一串回绕在 ±π 的角度展开后，返回**累计变化量**。

    直接用 `y[-1] - y[0]` 在跨过 ±π 时会得到 2π 量级的错误跳变，
    所以逐点把差值折进 (-π, π] 再累加。
    """
    total = 0.0
    for a, b in zip(yaws, yaws[1:]):
        d = b - a
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        total += d
    return total


def main():
    rclpy.init()
    f = Failures()
    print("=" * 72)
    print("ACG720 硬件在环（HIL）：真驱动节点 + 真 PTY + 真固件行为模型")
    print("=" * 72)

    # 前几项共用一个 harness（快）；涉及不同 scale/超时/故障的各起一个
    h = Harness(driver_scale=1.0)
    try:
        for name, fn in [
            ("链路与握手", test_link_come_up),
            ("直行与里程计", test_move_and_odom),
            ("旋转方向", test_rotation_direction),
            ("未标定里程计", test_odom_scale_uncorrected),
            ("命令超时停车", test_cmd_timeout_stops),
            ("运动期间不发心跳", test_no_heartbeat_during_motion),
        ]:
            try:
                fn(f, h)
            except Exception as exc:
                f.check(f"{name} 抛异常", False, repr(exc))
    finally:
        h.close()

    for name, fn in [
        ("标定后里程计", test_odom_scale_corrected),
        ("3 秒运行上限", test_run_limit_is_observed_and_reported),
        ("超限目标拦截", test_overspeed_is_refused_locally),
        ("驱动重启 SEQ 对齐", test_seq_resync_after_driver_restart),
        ("短掉线可恢复", test_short_glitch_is_survivable),
        ("长掉线锁死与复位恢复", test_long_glitch_latches_and_recovers_after_reset),
        ("位流不符拒绝运动", test_build_id_mismatch_refuses_motion),
        ("位置遥测链路", test_position_telemetry_surfaces),
    ]:
        try:
            fn(f)
        except Exception as exc:
            f.check(f"{name} 抛异常", False, repr(exc))

    print("\n" + "=" * 72)
    if f.items:
        print(f"失败 {len(f.items)} 项：")
        for i in f.items:
            print(f"  - {i}")
        print("=" * 72)
        return 1
    print("全部通过 ✓")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
