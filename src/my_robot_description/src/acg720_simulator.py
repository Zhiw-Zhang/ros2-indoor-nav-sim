#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""虚拟 ACG720 底盘：把 `Acg720Firmware` 接到 PTY 上，没有硬件也能端到端跑通。

它做什么
--------
创建一个 PTY（伪终端）对：

    虚拟车（master fd）  ←──── 内核里连着 ────→  /dev/pts/N（驱动节点打开）

然后把 `Acg720Driver(port='/dev/pts/N')` 指过来，两边就用**真实的 PTY、
真实的 pyserial 配置（115200 8N1）、真实的字节流**通信。所以这一层真能验证：

    ✅ 帧编解码 / CRC / 流式重新同步（走真字节流和真分帧逻辑）
    ✅ 驱动的串口打开、读写、断线重连路径
    ✅ /cmd_vel → MOVE → /odom + TF 全链路，含命令超时停车
    ✅ 运动学符号与尺度（原地转往哪边、scale 怎么影响转角）
    ✅ 真固件会咬人的那些行为：非法帧锁死、ACK 单槽、3.0 s 上限、故障自锁

    ❌ 不验证：电气、真实电机响应、真实编码器、真实轮胎摩擦、EMI。
       这些只有上车才知道，本文件不去假装。

与上一版的区别
--------------
上一版里"虚拟车"是本项目**自己设计**的行为模型，而且说的是自己发明的协议。
现在：**协议**由 `acg720_protocol_v1.py` 给出（与厂商实现逐字节交叉核对），
**行为**由 `acg720_firmware.py` 给出（按 RTL 逐条复刻）。
本文件退化成一层薄适配：PTY ↔ 模型，外加真值位姿发布。

用法
----
    # 拉起虚拟车 + 驱动节点（一条命令跑通全链路）
    ros2 run my_robot_description acg720_simulator

    # 只起虚拟车，打印 /dev/pts/N，自己手动把驱动指过去
    ros2 run my_robot_description acg720_simulator --no-driver

    # 自检（不连 ROS，验证 PTY 双向通道 + 协议 + 固件的危险行为）
    python3 src/my_robot_description/src/acg720_simulator.py --dry-run
"""

from __future__ import annotations

import argparse
import math
import os
import select
import sys
import termios
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from acg720_firmware import (  # noqa: E402
    Acg720Firmware, FirmwareParams, SafetyState, FaultReason, fault_cn,
)
from acg720_protocol_v1 import (  # noqa: E402
    Kind, FrameParser, SequenceTracker, encode_command, pack_move, pack_position,
    parse_diagnostic, parse_position_telemetry,
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
        except OSError:
            # 从机还没被打开（刚启动那几拍）时 master 写会返回 EIO。
            # 直接丢即可 —— 那几帧遥测本来就没人收。
            pass

    def close(self) -> None:
        for fd in (self.master_fd, self.slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass


class Acg720SimulatorNode(Node):
    """把 `Acg720Firmware` 接到 PTY 和 ROS 上。

    节点本身**不做任何协议/安全决策** —— 发什么帧、什么时候发，全部由固件模型
    决定。这一层只负责搬字节、记日志、发真值位姿。这样"虚拟车的行为"
    就只有唯一一处定义（`acg720_firmware.py`），不会被节点里的定时器带偏。
    """

    def __init__(self, pty: PtySerial, params: FirmwareParams,
                 physics_rate_hz: float = 100.0):
        super().__init__("acg720_simulator")
        self.pty = pty
        self.fw = Acg720Firmware(params)
        self.dt = 1.0 / physics_rate_hz
        self._last_fault: Optional[FaultReason] = None
        self._last_log = 0.0
        self._frames_in = 0
        self._rx_parser = FrameParser()      # 只为了统计"收到了多少帧"

        self.truth_pub = self.create_publisher(String, "acg720_sim/ground_truth", 10)
        self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f"虚拟 ACG720 已启动（按 RTL 复刻的真固件模型），PTY slave = {pty.slave_path}\n"
            f"  轮径 {params.wheel_radius_m*1000:.1f} mm  CPR {params.counts_per_rev:.0f}  "
            f"低速下限 {params.tracking_floor_rpm} RPM\n"
            f"  ★ 3.0 s 连续运动上限：{params.run_limit_s}s（会锁存 FAULT_RUN_LIMIT，"
            f"只有复位能解）\n"
            f"  ★ 300 ms 命令超时：{params.command_timeout_s}s\n"
            f"  真值有效轮距 {params.ground_truth_separation_m:.4f} m "
            f"(= 几何 {params.geometry_separation_m} m × "
            f"scale {params.ground_truth_separation_scale})\n"
            f"  把这行喂给驱动节点：--ros-args -p port:={pty.slave_path}"
        )

    # ------------------------------------------------------------------

    def _tick(self) -> None:
        # 1) 收下行字节 → 固件模型
        data = self.pty.read()
        if data:
            self._frames_in += len(self._rx_parser.feed(data))
            for event in self.fw.feed(data):
                if event not in ('accepted',):
                    self.get_logger().warn(
                        f"固件接收侧事件：{event}（crc_errors={self.fw.rx.crc_errors} "
                        f"duplicates={self.fw.rx.duplicates} "
                        f"frame_errors={self.fw.rx.frame_errors}）",
                        throttle_duration_sec=1.0)

        # 2) 推进固件模型，取回本该发出去的字节
        out = self.fw.tick(self.dt) + self.fw.drain_tx()
        if out:
            self.pty.write(out)

        # 3) 故障要**吵**。锁存 FAULT 之后车不会动了，现场必须一眼看出来
        fault = self.fw.fault
        if self.fw.state == SafetyState.FAULT and fault != self._last_fault:
            self.get_logger().error(
                f"★ 虚拟底盘已锁死：{fault.name}（{fault_cn(fault)}）。"
                f"真机上这只能靠复位解除 —— 驱动不该再发运动命令。")
        self._last_fault = fault if self.fw.state == SafetyState.FAULT else None

        # 4) 周期性把"真值"打出来。真机上没有这个东西，
        #    但它是**标定 scale** 的依据，所以虚拟车必须提供。
        now = self.fw.t
        if now - self._last_log >= 1.0:
            self._last_log = now
            p = self.fw.plant
            self.truth_pub.publish(String(data=(
                f"t={now:.1f}s pos=({p.x:.3f},{p.y:.3f}) "
                f"yaw={math.degrees(p.yaw):+.1f}deg "
                f"state={self.fw.state.name} fault={self.fw.fault.name} "
                f"counts={p.position_counts} rpm={p.measured_rpm} "
                f"frames_in={self._frames_in}"
            )))


# ==========================================================================
# 入口
# ==========================================================================

def parse_args(argv=None):
    p = argparse.ArgumentParser(description="虚拟 ACG720 底盘（PTY + 真固件行为模型）")
    p.add_argument("--no-driver", action="store_true",
                   help="只起虚拟车，不自动拉起驱动节点")
    p.add_argument("--dry-run", action="store_true",
                   help="不初始化 ROS，只建 PTY 并自检（用来隔离排查）")
    p.add_argument("--port", default=None, help="复用已存在的 PTY slave（调试用）")
    p.add_argument("--physics-rate", type=float, default=100.0,
                   help="虚拟车内部推进频率；100 Hz 与真固件的 50 ms 采样配合良好")
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument("--counts-per-rev", type=float, default=1320.0)
    p.add_argument("--wheel-radius", type=float, default=0.0325)
    p.add_argument("--tracking-floor-rpm", type=float, default=7.0,
                   help="复刻真车低速下限（交接包 F1 实测约 7 RPM）")
    p.add_argument("--gt-scale", type=float, default=1.0,
                   help="真值侧滑系数。1.0=无侧滑；标定实验里可调大以复现"
                        "'命令转 1 圈、车只转 0.73 圈'")
    p.add_argument("--gt-separation", type=float, default=0.200)
    p.add_argument("--run-limit-s", type=float, default=3.0,
                   help="★ 只给测试放宽用；默认就是固件真值 3.0 s。"
                        "在这里放宽等于把要验证的那条限制关掉")
    p.add_argument("--command-timeout-s", type=float, default=0.3)
    p.add_argument("--imu-available", action="store_true",
                   help="默认没有 IMU：0x82 会照发但 flags 报 stale，不伪造数据")
    p.add_argument("--seconds", type=float, default=0.0,
                   help="dry-run 模式下跑多少秒后退出（0=一直跑）")
    return p.parse_args(argv)


def _params_from_args(args) -> FirmwareParams:
    return FirmwareParams(
        baudrate=args.baudrate,
        counts_per_rev=args.counts_per_rev,
        wheel_radius_m=args.wheel_radius,
        geometry_separation_m=args.gt_separation,
        ground_truth_separation_scale=args.gt_scale,
        tracking_floor_rpm=args.tracking_floor_rpm,
        run_limit_s=args.run_limit_s,
        command_timeout_s=args.command_timeout_s,
        imu_available=args.imu_available,
    )


def main(argv=None) -> int:
    args = parse_args(argv)

    pty = PtySerial()
    print(f"[虚拟 ACG720] PTY slave 路径：{pty.slave_path}")

    if args.dry_run:
        code = _dry_run(args, pty)
        pty.close()
        return code

    rclpy.init()
    node = Acg720SimulatorNode(pty, _params_from_args(args),
                               physics_rate_hz=args.physics_rate)
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


# ==========================================================================
# 自检：不依赖 ROS，把"固件最会咬人的几条"逐条证明一遍
# ==========================================================================

class _MiniHost:
    """自检用的小主机：管 SEQ、解析回帧、按周期推进。"""

    def __init__(self, fw: Acg720Firmware):
        self.fw = fw
        self.seq = SequenceTracker()
        self.parser = FrameParser()
        self.frames: List[tuple] = []

    def send(self, kind, payload: bytes = b'') -> List[str]:
        return self.fw.feed(encode_command(kind, self.seq.take(), payload))

    def drain(self) -> bytes:
        data = self.fw.drain_tx()
        self.frames.extend(self.parser.feed(data))
        return data

    def tick(self, dt: float) -> bytes:
        data = self.fw.tick(dt)
        self.frames.extend(self.parser.feed(data))
        return data

    def of(self, kind) -> List[tuple]:
        return [f for f in self.frames if f[0] == int(kind)]


def _dry_run(args, pty: PtySerial) -> int:
    print("[dry-run] 不初始化 ROS，验证 PTY 双向通道 + 固件行为…")
    failures = 0

    def check(name: str, ok: bool, detail: str = "") -> None:
        nonlocal failures
        print(f"  {'✓' if ok else '✗'} {name}" + (f"  {detail}" if detail else ""))
        if not ok:
            failures += 1

    # ---- ① 方向：虚拟车 → 驱动（把固件的遥测发到 slave 端）----
    pty.write(encode_command(Kind.HEARTBEAT, 0))     # 任意合法帧，证明通道通
    got = pty.read_from_slave(timeout=1.0)
    check("方向① 虚拟车→驱动（master 写、slave 读）", bool(got),
          f"{len(got)} 字节")

    # ---- ② 方向：驱动 → 虚拟车（slave 写、master 读）----
    #   ★ 必须是 write_to_slave（扮演驱动），不能是 pty.write（那是虚拟车）。
    #   ★ 必须给非零 timeout：PTY 写入到对端可读之间有延迟。
    fw = Acg720Firmware(_params_from_args(args))
    host = _MiniHost(fw)
    # 这一帧既是"方向②"的探针，也是真的 ARM。⚠ 必须走 host.seq 取序号，
    #   否则后面 host.send() 会拿到同一个 SEQ 而被固件判成重复（静默丢弃）。
    pty.write_to_slave(encode_command(Kind.ARM, host.seq.take()))
    back = pty.read(timeout=1.0)
    events = fw.feed(back) if back else []
    check("方向② 驱动→虚拟车（slave 写、master 读）", bool(back),
          f"{len(back)} 字节，事件 {events}")

    # ---- ③ ARM → MOVE：状态机走通，且 ACK 回得来 ----
    host.tick(0.02)
    host.drain()
    events = host.send(Kind.MOVE, pack_move(1, 100))
    for _ in range(10):
        host.tick(0.02)
    host.drain()
    check("ARM 后 MOVE 进入 RUNNING", fw.state == SafetyState.RUNNING,
          f"state={fw.state.name} events={events}")
    check("收到了 ACK", len(host.of(Kind.ACK)) >= 1,
          f"{len(host.of(Kind.ACK))} 帧")
    check("收到了 0x83 诊断帧且 build_id 正确",
          any(parse_diagnostic(f[2])['build_id_hex'] == '20260A02'
              for f in host.of(Kind.DIAGNOSTIC)))

    # ---- ④ 非法帧（VERSION 错）必须锁死 ----
    fw2 = Acg720Firmware(_params_from_args(args))
    bad = bytearray(encode_command(Kind.HEARTBEAT, 0))
    bad[2] = 0x02
    fw2.feed(bytes(bad))
    fw2.tick(0.02)
    check("VERSION 错的帧锁死底盘（PROTOCOL）",
          fw2.state == SafetyState.FAULT and fw2.fault == FaultReason.PROTOCOL,
          f"state={fw2.state.name} fault={fw2.fault.name}")

    # ---- ⑤ 3.0 s 连续运动上限：连 10 Hz 重发 MOVE 也躲不过 ----
    fw3 = Acg720Firmware(_params_from_args(args))
    host3 = _MiniHost(fw3)
    host3.send(Kind.ARM)
    host3.tick(0.02)
    t = 0.0
    next_at = 0.0
    while t < fw3.p.run_limit_s + 0.6:
        if t >= next_at - 1e-12:
            host3.send(Kind.MOVE, pack_move(1, 100))
            next_at += 0.1
        host3.tick(0.02)
        t += 0.02
    check(f"连续运动 {fw3.p.run_limit_s}s 后锁死 RUN_LIMIT",
          fw3.state == SafetyState.FAULT and fw3.fault == FaultReason.RUN_LIMIT,
          f"state={fw3.state.name} fault={fw3.fault.name}")

    # ---- ⑥ 位置环：走完并报 OK（且不受 3 s 上限约束）----
    fw4 = Acg720Firmware(_params_from_args(args))
    host4 = _MiniHost(fw4)
    host4.send(Kind.ARM)
    host4.tick(0.02)
    host4.send(Kind.POSITION, pack_position(1, 25, 3232))
    t = 0.0
    next_at = 0.0
    progress = []
    while t < 12.0 and fw4.position.busy:
        if t >= next_at - 1e-12:
            host4.send(Kind.HEARTBEAT)
            next_at += 0.1
        host4.tick(0.02)
        t += 0.02
        progress.append(fw4.position.progress_counts)
    ok = fw4.position.result_code.name == 'OK'
    check("位置动作走完并报 OK（>3 s 也不触发上限）", ok,
          f"progress={fw4.position.progress_counts} t={t:.2f}s "
          f"result={fw4.position.result_code.name}")
    check("位置遥测 0x84 报得出进度",
          any(parse_position_telemetry(f[2])['target_counts'] == 3232
              for f in host4.of(Kind.POSITION_TELEMETRY)))

    # ---- ⑦ 低速滑行下限（真车缺陷）必须复现 ----
    fw5 = Acg720Firmware(_params_from_args(args))
    host5 = _MiniHost(fw5)
    host5.send(Kind.ARM)
    host5.tick(0.02)
    t = 0.0
    next_at = 0.0
    while t < 1.5:
        if t >= next_at - 1e-12:
            host5.send(Kind.MOVE, pack_move(1, 3))
            next_at += 0.1
        host5.tick(0.02)
        t += 0.02
    steady = fw5.plant.rpm_f[0]
    check(f"目标 3 RPM（低于下限）→ 实际稳定在 ≈{fw5.p.tracking_floor_rpm} RPM",
          abs(steady - fw5.p.tracking_floor_rpm) < 0.5, f"{steady:.2f} RPM")

    if failures:
        print(f"[dry-run] 自检失败 {failures} 项")
        return 1
    print("[dry-run] 自检全部通过。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
