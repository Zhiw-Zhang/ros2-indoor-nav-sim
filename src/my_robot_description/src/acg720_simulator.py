#!/usr/bin/env python3
"""虚拟 ACG720 底盘：让 ROS 2 侧在**没有硬件**的情况下端到端跑通。

它做什么
--------
创建一个 PTY（伪终端）对：

    虚拟车（master fd）  ←──── 内核里连着 ────→  /dev/pts/N（驱动节点打开）

然后把 `Acg720Driver(port='/dev/pts/N')` 指过来，两边就用**真实的 PTY、
真实的 pyserial 配置（115200 8N1）、真实的字节流**通信。所以这一层真能验证：

    ✅ 帧编解码 / CRC / 流式重新同步（走真字节流和真分帧逻辑）
    ✅ 驱动的串口打开、读写、断线重连路径
    ✅ /cmd_vel → 四轮 RPM → /odom + TF 全链路，含命令超时停车
    ✅ 运动学符号与尺度（原地转往哪边、scale 怎么影响转角）
    ✅ 真车已知的两个坑：7 RPM 低速走不动、skid 侧滑

    ❌ 不验证：电气、真实电机响应、真实编码器、真实轮胎摩擦、EMI。
       这些只有上车才知道，本文件不去假装。

刻意复刻真车的已知缺陷（依据 = 交接包 F1 验收报告）
--------------------------------------------------
1. **内轮目标低于约 7 RPM 时无法稳定跟踪**，轮子以约 7 RPM 滑行
   → `tracking_floor_rpm`（默认 7.0）。
   整个避障减速段都落在这个区间，所以这个缺陷会直接影响真机行为，
   必须在没硬件时就暴露出来，而不是上了车才发现"让它慢慢挪它却冲出去"。

2. **skid-steer 侧滑**：车体实际转角小于差速模型预测
   → `ground_truth_separation_scale`（默认 1.36）代表"真实车体"的有效轮距。
   驱动侧默认 1.0，所以**默认配置下就能看到"命令转 1 圈、车实际只转 0.73 圈"**；
   把驱动的 scale 标到 1.36，误差就被收掉了 —— 这正是真机要走的标定流程。

3. **FPGA 拒绝 > 200 RPM 的目标**（交接包 4.3 节）
   → `max_target_rpm`；被拒时回 0x83 状态帧带拒绝码，驱动会打 WARN。

4. **心跳 300 ms 超时进安全态**（交接包 4.4 节）→ `heartbeat_timeout`。

用法
----
    # 拉起虚拟车 + 驱动节点（一条命令跑通全链路）
    ros2 run my_robot_description acg720_simulator

    # 只起虚拟车，打印 /dev/pts/N，自己手动把驱动指过去
    ros2 run my_robot_description acg720_simulator --no-driver

    # 干跑（不连 ROS，只验证 PTY + 协议）
    python3 src/my_robot_description/src/acg720_simulator.py --no-driver --dry-run
"""

from __future__ import annotations

import argparse
import math
import os
import select
import struct
import sys
import termios
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from acg720_protocol import (  # noqa: E402
    Cmd, Flag, Frame, FrameParser, PositionDone, Status, Telemetry,
    encode, lr_to_wheels, pack_position_done, pack_status, pack_telemetry,
    parse_heartbeat, parse_position, parse_status, parse_telemetry,
    parse_velocity, parse_position_done, wheels_to_lr,
)


class PtySerial:
    """把 PTY 的 master fd 包装成"可读写的串口"。

    为什么不用 pyserial 打开 master：pyserial 需要设备路径，而 master fd
    没有路径（只有 slave 有）。所以这里直接用 os.read/os.write，
    并把 master 设成 raw 模式，行为就等价于一个 115200 的串口——
    实际上波特率在 PTY 上是**不生效**的（内核不做分频），但这不影响
    本层的验证目标：帧格式、分帧、流控行为都是真的。
    """

    def __init__(self):
        self.master_fd, self.slave_fd = os.openpty()
        self.slave_path = os.ttyname(self.slave_fd)
        # master 设 raw：否则终端的行规程（回车换行转换、Ctrl-C 信号）
        # 会把二进制帧搞坏 —— 这是 PTY 做二进制通道最常见的坑
        attrs = termios.tcgetattr(self.master_fd)
        attrs[0] = 0                                     # iflag
        attrs[1] = 0                                     # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL   # cflag
        attrs[3] = 0                                     # lflag
        attrs[6][termios.VMIN] = 0                       # 非阻塞读
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.master_fd, termios.TCSANOW, attrs)
        os.set_blocking(self.master_fd, False)

    def read(self, max_bytes: int = 4096, timeout: float = 0.0) -> bytes:
        """从 master 读（FPGA 视角：接收上位机命令）。

        ⚠ `timeout=0` 是**竞态**的：刚往 slave 写完立刻 select master，
        内核里那一瞬间可能还看不到数据，于是"什么都没读到"，
        而调用方会误判成"链路断了"。自检和测试里必须给一个非零 timeout。
        """
        r, _, _ = select.select([self.master_fd], [], [], timeout)
        if not r:
            return b""
        try:
            return os.read(self.master_fd, max_bytes)
        except BlockingIOError:
            return b""
        except OSError:
            return b""

    def read_from_slave(self, max_bytes: int = 4096, timeout: float = 0.5) -> bytes:
        """从 slave 端读。**只给自检用**。

        ⚠ 一个常见误解：PTY **不是回环**。往 master 写，字节去 slave；
        往 slave 写，字节去 master。所以"往 master 写再从 master 读"永远读不到
        东西 —— 一个写错方向的自检会**静默通过**（什么都没读到，什么也没断言）。
        这里显式从 slave 读，就是为了让自检真的证明通道是通的。
        """
        r, _, _ = select.select([self.slave_fd], [], [], timeout)
        if not r:
            return b""
        try:
            return os.read(self.slave_fd, max_bytes)
        except (BlockingIOError, OSError):
            return b""

    def write_to_slave(self, data: bytes) -> int:
        """从 slave 端写。**只给自检用**（扮演"驱动节点往串口写"）。

        ⚠⚠ 这是本文件踩过的坑，写下来免得再犯：
        往 master 写 = 虚拟车发给驱动；往 slave 写 = 驱动发给虚拟车。
        两个方向**必须用不同的 fd**。早先的自检里两个方向都用了 `pty.write()`
        （即都走 master），结果方向② 永远读到 0 字节，而测试把它当成
        "链路断了"，白白排查了半天。所以这里给方向② 一个**显式命名**的入口，
        让"我到底在扮演哪一端"在调用处就能看出来。
        """
        return os.write(self.slave_fd, data)

    def write(self, data: bytes) -> None:
        try:
            os.write(self.master_fd, data)
        except BlockingIOError:
            pass   # PTY 缓冲满：直接丢，和真串口丢字节的行为一致

    def close(self) -> None:
        for fd in (self.master_fd, self.slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass


@dataclass
class WheelState:
    """单个轮子的状态。"""
    position_counts: float = 0.0     # 累计计数（物理真值，浮点，上报时取整）
    rpm: float = 0.0                 # 当前实际转速
    #: 上一次已上报的累计计数。上报的是**区间增量**，所以要对齐。
    reported_counts: float = field(default=0.0, repr=False)


class VirtualAcg720:
    """ACG720 的行为模型。

    时间推进由外部按固定步长调用 `step(dt)`，这样物理是确定性的：
    同样的指令序列一定得到同样的计数和位姿，测试才不会 flaky。
    """

    def __init__(self,
                 wheel_radius_m: float = 0.0325,
                 counts_per_rev: float = 1320.0,
                 tracking_floor_rpm: float = 7.0,
                 max_target_rpm: float = 200.0,
                 ground_truth_separation_m: float = 0.200,
                 ground_truth_separation_scale: float = 1.36,
                 motor_time_constant_s: float = 0.12,
                 heartbeat_timeout_s: float = 0.30,
                 telemetry_period_s: float = 0.05,
                 seed: int = 0):
        self.wheel_radius_m = wheel_radius_m
        self.counts_per_rev = counts_per_rev
        self.tracking_floor_rpm = tracking_floor_rpm
        self.max_target_rpm = max_target_rpm
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self.telemetry_period_s = telemetry_period_s
        self.motor_tau = motor_time_constant_s

        #: "真实车体"的有效轮距。物理上决定车实际转多快。
        #: 它和驱动侧用的 b_eff 不一样时，就能看到转角误差 —— 这就是标定的意义。
        self.gt_separation = ground_truth_separation_m * ground_truth_separation_scale

        # 四个轮子：A=右前 B=左前 C=左后 D=右后（交接包的轮位约定）
        self.wheels = [WheelState() for _ in range(4)]
        self.target_rpm = [0.0, 0.0, 0.0, 0.0]

        # 车体真实位姿（虚拟车里存在，真机上不存在）
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.flags = 0
        self.uptime_s = 0.0
        self.seq = 0
        self.last_heartbeat_s = -1e9
        self.reject_code = 0
        self.accepted = True

        # 卡死检测：duty 饱和且实测 <=3 RPM 连续 10 个采样
        self._stall_samples = 0
        self._duty = [0, 0, 0, 0]
        self._last_reject_reason = 0

    # ---------------- 上报 ----------------

    @property
    def counts_per_meter(self) -> float:
        return self.counts_per_rev / (2.0 * math.pi * self.wheel_radius_m)

    @property
    def rpm_to_mps(self) -> float:
        return 2.0 * math.pi * self.wheel_radius_m / 60.0

    def left_right_rpm(self) -> Tuple[float, float]:
        return wheels_to_lr(*self.wheels_rpm())

    def wheels_rpm(self) -> Tuple[float, float, float, float]:
        return tuple(w.rpm for w in self.wheels)

    def make_telemetry(self) -> Telemetry:
        """组一帧 0x84。counts 是**本采样区间的增量**（带符号）。"""
        counts = []
        for w in self.wheels:
            delta = int(round(w.position_counts - w.reported_counts))
            w.reported_counts += delta
            counts.append(delta)
        rpm = tuple(int(round(w.rpm)) for w in self.wheels)
        return Telemetry(seq=self.seq, uptime_ms=int(self.uptime_s * 1000.0),
                         counts=tuple(counts), rpm=rpm,
                         duty=tuple(self._duty), flags=self.flags)

    def make_status(self, heartbeat_echo: int) -> Status:
        return Status(heartbeat_echo=heartbeat_echo, accepted=self.accepted,
                      reject_code=self.reject_code,
                      uptime_ms=int(self.uptime_s * 1000.0))

    # ---------------- 下行命令 ----------------

    def apply_velocity(self, payload: bytes) -> None:
        cmd = parse_velocity(payload)
        self.accepted = True
        self.reject_code = 0
        # 复刻 FPGA 的保护：目标超限直接拒（交接包 4.3 节，>200 RPM 判故障）
        for v in cmd.rpm:
            if abs(v) > self.max_target_rpm:
                self.accepted = False
                self.reject_code = 1        # TARGET_GT_200RPM
                self.flags |= Flag.TARGET_REJECT
                return
        self.target_rpm = [float(v) for v in cmd.rpm]
        self.flags &= ~Flag.TARGET_REJECT

    def apply_position(self, payload: bytes) -> None:
        cmd = parse_position(payload)
        # 位置环在本模型里不做闭环，只记录 + 立刻回报"被新命令打断"，
        # 免得假装验证了一个还没实现的环。
        self._pending_position = cmd

    def note_heartbeat(self, payload: bytes) -> int:
        seq = parse_heartbeat(payload)
        self.last_heartbeat_s = self.uptime_s
        self.flags &= ~Flag.HEARTBEAT_TIMEOUT
        return seq

    # ---------------- 物理推进 ----------------

    def step(self, dt: float) -> None:
        self.uptime_s += dt

        # 心跳超时 → 安全态（交接包 4.4 节：300 ms）
        if (self.uptime_s - self.last_heartbeat_s) > self.heartbeat_timeout_s:
            self.flags |= Flag.HEARTBEAT_TIMEOUT
            self.target_rpm = [0.0, 0.0, 0.0, 0.0]

        commanded_mags = [abs(v) for v in self.target_rpm if abs(v) > 1e-9]

        for i, w in enumerate(self.wheels):
            tgt = self.target_rpm[i]

            # ★ 低速跟踪下限：非零但小于下限的目标，车实际以"下限"滑行。
            #   这不是 bug，是交接包 F1 验收里实测到的真实行为。
            if 1e-9 < abs(tgt) < self.tracking_floor_rpm:
                effective = math.copysign(self.tracking_floor_rpm, tgt)
                self.flags |= Flag.LOW_SPEED_WARN
            else:
                effective = tgt

            # 一阶电机响应：τ 约 120 ms（拍值，真机待实测）
            alpha = 1.0 - math.exp(-dt / self.motor_tau) if self.motor_tau > 0 else 1.0
            w.rpm += (effective - w.rpm) * alpha
            if abs(w.rpm) < 1e-6:
                w.rpm = 0.0

            # duty 近似：前馈零点约 150 + 斜率，上限 2700（54%）
            self._duty[i] = int(min(2700, max(0, 150 + abs(w.rpm) * 12.0)))

        # 低速警告旗标是"瞬时"的，本采样没有低速目标就清掉
        if not any(1e-9 < m < self.tracking_floor_rpm for m in commanded_mags):
            self.flags &= ~Flag.LOW_SPEED_WARN

        # 卡死：duty 饱和 + 实测几乎不转
        if all(d >= 1800 for d in self._duty) and all(abs(w.rpm) <= 3.0 for w in self.wheels):
            self._stall_samples += 1
        else:
            self._stall_samples = 0
        if self._stall_samples >= 10:
            self.flags |= Flag.STALL

        # --- 编码器计数 ---
        for w in self.wheels:
            w.position_counts += w.rpm * self.counts_per_rev / 60.0 * dt

        # --- 车体真实位姿：用"真实有效轮距"，所以会体现侧滑 ---
        left_rpm, right_rpm = self.left_right_rpm()
        v_l = left_rpm * self.rpm_to_mps
        v_r = right_rpm * self.rpm_to_mps
        v = (v_l + v_r) / 2.0
        omega = (v_r - v_l) / self.gt_separation          # ★ 真值侧滑在这里
        yaw_mid = self.yaw + omega * dt / 2.0
        self.x += v * math.cos(yaw_mid) * dt
        self.y += v * math.sin(yaw_mid) * dt
        self.yaw = (self.yaw + omega * dt + math.pi) % (2.0 * math.pi) - math.pi
        self.seq = (self.seq + 1) & 0xFFFFFFFF


class Acg720SimulatorNode(Node):
    """把 VirtualAcg720 接到 PTY 和 ROS 上。"""

    def __init__(self, pty: PtySerial, *, physics_rate_hz: float = 50.0,
                 telemetry_rate_hz: float = 20.0, **model_kwargs):
        super().__init__("acg720_simulator")
        self.pty = pty
        self.model = VirtualAcg720(**model_kwargs)
        self.parser = FrameParser()
        self.physics_dt = 1.0 / physics_rate_hz
        self.telemetry_period = 1.0 / telemetry_rate_hz
        self._since_telemetry = 0.0
        self._last_heartbeat_echo = 0
        self._frames_in = 0
        self._bad_frames = 0
        self._last_log = 0.0

        self.truth_pub = self.create_publisher(String, "acg720_sim/ground_truth", 10)
        self.create_timer(self.physics_dt, self._tick)

        self.get_logger().info(
            f"虚拟 ACG720 已启动，PTY slave = {pty.slave_path}\n"
            f"  轮径 {self.model.wheel_radius_m*1000:.1f} mm  "
            f"CPR {self.model.counts_per_rev:.0f}  "
            f"低速下限 {self.model.tracking_floor_rpm} RPM\n"
            f"  真值有效轮距 {self.model.gt_separation:.4f} m "
            f"(= 几何 {model_kwargs.get('ground_truth_separation_m', 0.2)} m × "
            f"scale {model_kwargs.get('ground_truth_separation_scale', 1.36)})\n"
            f"  把这行喂给驱动节点：--ros-args -p port:={pty.slave_path}"
        )

    def _tick(self) -> None:
        # 1) 收下行帧
        data = self.pty.read()
        if data:
            for frame in self.parser.feed(data):
                self._frames_in += 1
                try:
                    self._dispatch(frame)
                except ValueError as exc:
                    self._bad_frames += 1
                    self.get_logger().warn(f"下行帧解析失败：{exc}")

        # 2) 推进物理
        self.model.step(self.physics_dt)

        # 3) 按 20 Hz 上报遥测
        self._since_telemetry += self.physics_dt
        if self._since_telemetry >= self.telemetry_period:
            self._since_telemetry = 0.0
            self.pty.write(encode(Cmd.TELEMETRY, pack_telemetry(self.model.make_telemetry())))
            self.pty.write(encode(Cmd.STATUS,
                                  pack_status(self.model.make_status(self._last_heartbeat_echo))))

        # 4) 周期性把"真值"打出来。真机上没有这个东西，
        #    但它是**标定 scale 的依据**，所以虚拟车必须提供。
        now = self.model.uptime_s
        if now - self._last_log >= 1.0:
            self._last_log = now
            l_rpm, r_rpm = self.model.left_right_rpm()
            self.truth_pub.publish(String(data=(
                f"t={now:.1f}s pos=({self.model.x:.3f},{self.model.y:.3f}) "
                f"yaw={math.degrees(self.model.yaw):+.1f}deg "
                f"wheels_rpm=({self.model.wheels[0].rpm:.1f},{self.model.wheels[1].rpm:.1f},"
                f"{self.model.wheels[2].rpm:.1f},{self.model.wheels[3].rpm:.1f}) "
                f"L={l_rpm:.1f} R={r_rpm:.1f} flags=0x{self.model.flags:04X} "
                f"frames_in={self._frames_in}"
            )))

    def _dispatch(self, frame: Frame) -> None:
        if frame.cmd == Cmd.SET_VELOCITY:
            self.model.apply_velocity(frame.payload)
        elif frame.cmd == Cmd.HEARTBEAT:
            self._last_heartbeat_echo = self.model.note_heartbeat(frame.payload)
        elif frame.cmd == Cmd.POSITION:
            self.model.apply_position(frame.payload)
            done = PositionDone(mode=0, success=False, reason=3,   # PREEMPTED
                                final_counts=0, elapsed_samples=0)
            self.pty.write(encode(Cmd.POSITION_DONE, pack_position_done(done)))
        else:
            self.get_logger().warn(f"虚拟车收到未知命令字 0x{frame.cmd:02X}")


# ==========================================================================
# 入口
# ==========================================================================

def parse_args(argv=None):
    p = argparse.ArgumentParser(description="虚拟 ACG720 底盘（PTY + 协议模型）")
    p.add_argument("--no-driver", action="store_true",
                   help="只起虚拟车，不自动拉起驱动节点")
    p.add_argument("--dry-run", action="store_true",
                   help="不初始化 ROS，只建 PTY 并打印路径（用来隔离排查）")
    p.add_argument("--port", default=None,
                   help="复用已存在的 PTY slave（调试用）")
    p.add_argument("--physics-rate", type=float, default=50.0)
    p.add_argument("--telemetry-rate", type=float, default=20.0)
    p.add_argument("--tracking-floor-rpm", type=float, default=7.0,
                   help="复刻真车低速下限（交接包 F1 实测约 7 RPM）")
    p.add_argument("--gt-scale", type=float, default=1.36,
                   help="真值侧滑系数，即仿真真车的有效轮距倍率")
    p.add_argument("--gt-separation", type=float, default=0.200)
    p.add_argument("--counts-per-rev", type=float, default=1320.0)
    p.add_argument("--wheel-radius", type=float, default=0.0325)
    p.add_argument("--seconds", type=float, default=0.0,
                   help="dry-run 模式下跑多少秒后退出（0=一直跑）")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    pty = PtySerial()
    print(f"[虚拟 ACG720] PTY slave 路径：{pty.slave_path}")

    if args.dry_run:
        # 不依赖 ROS 的最小通道自检：
        #   ① master 写 → slave 读   （遥测方向：虚拟车 → 驱动）
        #   ② slave 写 → master 读   （命令方向：驱动 → 虚拟车）
        # 两个方向都要过，才能真正证明 PTY 通道可用。
        print("[dry-run] 不初始化 ROS，验证 PTY 双向通道 + 协议编解码…")
        parser = FrameParser()
        model = VirtualAcg720(
            wheel_radius_m=args.wheel_radius,
            counts_per_rev=args.counts_per_rev,
            tracking_floor_rpm=args.tracking_floor_rpm,
            ground_truth_separation_m=args.gt_separation,
            ground_truth_separation_scale=args.gt_scale,
        )
        failures = 0

        # ---- 方向 ①：虚拟车 → 驱动（遥测 + 状态）----
        for i in range(3):
            model.step(0.05)
            sent = encode(Cmd.TELEMETRY, pack_telemetry(model.make_telemetry()))
            pty.write(sent)
        got = pty.read_from_slave(timeout=1.0)
        if not got:
            print("  ✗ 方向① 失败：往 master 写了遥测，slave 端一个字节都没收到")
            failures += 1
        else:
            frames = parser.feed(got)
            tele = [f for f in frames if f.cmd == Cmd.TELEMETRY]
            if len(tele) != 3:
                print(f"  ✗ 方向① 失败：期望 3 帧遥测，实收 {len(tele)} 帧")
                failures += 1
            else:
                print(f"  ✓ 方向① 虚拟车→驱动：{len(tele)} 帧遥测，"
                      f"{len(got)} 字节，最后一帧 "
                      f"counts={parse_telemetry(tele[-1].payload).counts}")

        # ---- 方向 ②：驱动 → 虚拟车（速度命令）----
        #   ★ 这一行必须是 write_to_slave（扮演驱动），不能是 pty.write（那是虚拟车）。
        #     写成 pty.write 的话字节会去 slave 端，master 永远读不到。
        import acg720_protocol as _p
        pty.write_to_slave(encode(Cmd.SET_VELOCITY, _p.pack_velocity(30, 40, 40, 30)))
        #   ★ 必须给非零 timeout：PTY 写入到对端可读之间有延迟，
        #     timeout=0 的读会随机返回空，把"链路正常"误报成"链路断了"
        cmd_back = pty.read(timeout=1.0)
        if not cmd_back:
            print("  ✗ 方向② 失败：往 slave 写了速度命令，master 端没收到")
            failures += 1
        else:
            parsed = [f for f in parser.feed(cmd_back) if f.cmd == Cmd.SET_VELOCITY]
            if not parsed:
                print(f"  ✗ 方向② 失败：收到的字节不是合法的 SET_VELOCITY 帧："
                      f"{cmd_back.hex(' ')}")
                failures += 1
            else:
                v = parse_velocity(parsed[-1].payload)
                model.apply_velocity(parsed[-1].payload)
                print(f"  ✓ 方向② 驱动→虚拟车：解析出目标 RPM {v.rpm}，"
                      f"左/右 = {v.left_rpm}/{v.right_rpm}")

        # ---- ③ 物理模型：低速下限必须复现（这是真车已知缺陷）----
        #   ⚠ 必须每步喂心跳，否则 300 ms 心跳超时会把目标清零，
        #     模型永远停在 0 RPM，测试就变成"测了但什么都没测到"
        model2 = VirtualAcg720(wheel_radius_m=args.wheel_radius,
                               tracking_floor_rpm=args.tracking_floor_rpm)
        model2.target_rpm = [3.0, 3.0, 3.0, 3.0]     # 内轮典型低速目标
        for _ in range(100):                          # 5 s，足够进入稳态
            model2.last_heartbeat_s = model2.uptime_s
            model2.step(0.05)
        steady = model2.wheels[0].rpm
        if abs(steady - model2.tracking_floor_rpm) < 0.5:
            print(f"  ✓ 物理模型：目标 3 RPM（低于下限）→ 实际稳定在 "
                  f"{steady:.1f} RPM，复现了真车低速滑行")
        else:
            print(f"  ✗ 物理模型：期望稳定在 {model2.tracking_floor_rpm} RPM，"
                  f"实际 {steady:.1f} RPM")
            failures += 1

        # ---- ④ 物理模型：skid 侧滑必须让实际转角小于几何差速预测 ----
        model3 = VirtualAcg720(ground_truth_separation_scale=args.gt_scale,
                               ground_truth_separation_m=args.gt_separation)
        model3.target_rpm = [-20.0, 20.0, 20.0, -20.0]   # 左旋
        for _ in range(20):                               # 1 s
            model3.last_heartbeat_s = model3.uptime_s
            model3.step(0.05)
        # 实际角速度 = Δv / gt_separation（真值有效轮距）
        l_rpm, r_rpm = model3.left_right_rpm()
        delta_v = (r_rpm - l_rpm) * model3.rpm_to_mps
        actual = delta_v / model3.gt_separation
        # 几何差速预测 = Δv / 几何轮距（即 scale=1.0 时里程计会报的值）
        naive = delta_v / args.gt_separation
        ratio = actual / naive if naive else 0.0
        expected = 1.0 / args.gt_scale
        if abs(ratio - expected) < 1e-6:
            print(f"  ✓ 物理模型：实际转角 = 几何差速预测的 {ratio:.4f} 倍 "
                  f"(= 1/{args.gt_scale:.2f})，侧滑已生效；"
                  f"实测 yaw={math.degrees(model3.yaw):+.2f}°")
        else:
            print(f"  ✗ 物理模型：侧滑比值 {ratio:.4f}，期望 {expected:.4f}")
            failures += 1

        pty.close()
        if failures:
            print(f"[dry-run] 自检失败 {failures} 项")
            return 1
        print("[dry-run] 自检全部通过。")
        return 0

    rclpy.init()
    node = Acg720SimulatorNode(
        pty, physics_rate_hz=args.physics_rate,
        telemetry_rate_hz=args.telemetry_rate,
        wheel_radius_m=args.wheel_radius,
        counts_per_rev=args.counts_per_rev,
        tracking_floor_rpm=args.tracking_floor_rpm,
        ground_truth_separation_m=args.gt_separation,
        ground_truth_separation_scale=args.gt_scale,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        pty.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
