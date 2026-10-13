#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ACG720 固件行为模型 —— 按 FPGA RTL 逐条复刻，用来在没有硬件时撞到真问题。

为什么需要这个文件
==================
`acg720_protocol_v1.py` 解决的是"说什么话"（帧格式、CRC、载荷布局）。
本文件解决的是"**说完话之后固件会怎么做**"：

    · 哪一帧会被接受、哪一帧会被判非法并**永久锁死底盘**
    · ARM / 急停 / 300 ms 命令超时 / 3.0 s 连续运动上限
    · 位置环怎么走、怎么算完成、什么时候判堵转/超时
    · 遥测的 ACK 只有单槽，撞车会 `tx_overflow` → 也是永久锁死
    · 目标转速怎么展开成四个轮子，以及**哪些 (v, ω) 根本表达不出来**

没有它，我们只能在真车上一条条试，而**试错的代价是车锁死 + 拆电复位**。
有了它，这些行为都能在毫秒级的单测里钉死（见 `test_firmware_model.py`）。

刻意复刻、不是"设计"
====================
★ 本文件的原则是**照抄**，不是"做个合理的模型"。
  凡是 RTL 里看着像 bug 的地方（HEARTBEAT 会刹停运动、`run_age` 重发命令不清零、
  内侧轮不能反转、ACK 单槽撞车锁死）**一律照抄**，并在注释里给出 `文件:行号`。
  把它们"顺手修好"就等于把我们要发现的问题藏起来。

权威出处（全部在交接包 `robot_car_handoff_20261006` 里，**不在本仓库**）
======================================================================
  · .../output_file/chassis_command_rx.v         帧状态机、CRC、LEN 上限、SEQ、语义校验
  · .../output_file/chassis_safety_fsm.v         安全状态机、三个超时、故障锁存
  · .../output_file/chassis_position_controller.v 位置环（剖面/斜坡/堵转/超时/完成）
  · .../output_file/chassis_motion.v             动作码 → 四轮转速
  · .../output_file/chassis_telemetry_imu_tx.v   5 种遥测帧、ACK 单槽、tx_overflow
  · .../output_file/chassis_four_motor_control.v 电机通道（本模型只做一阶近似）
  · .../output_file/encoder_quadrature.v         50 ms 采样、RPM = (计数×238313)>>18
  · .../output_file/top.v                        模块互连：active_* / position_abort /
                                                 command_stop_target / position_busy 分路
  · 交接包 `小车数据总览_实测状态分级.md` F1 段   低速滑行下限≈7 RPM、skid 侧滑

时钟与时间基准
==============
RTL 是 50 MHz 同步设计（top.v:5,164）。本模型不模拟时钟周期，而是把每个
`*_CYCLES` 常数按 50 MHz 折算成秒（见 `FirmwareParams`），再用 `_advance(dt)`
统一推进。**唯一的时间来源是 dt**，不用墙上时钟——这样 HIL 测试不会 flaky。

`feed()` 里每个字节也推进 `10/波特率` 秒：真串口上一个字节就是占这么久的，
不推的话 ACK 单槽撞车（`tx_overflow`）的时序就不对了。

刻意不模拟的东西（写清楚，免得被误当成"已验证"）
================================================
  · 电气、PWM 死区、母线压降、EMI、编码器毛刺（`invalid_transition_count` 恒 0）
  · 电机通道故障（`motor_fault`）—— 只留 `inject_motor_fault()` 注入点，
    用来验证 FSM 的 FAULT_MOTOR 分支，默认不触发
  · JY901S IMU 硬件 —— 0x82 帧照发，但 flags 报 **stale/frame_timeout**，
    **绝不伪造"有效"的 IMU 数据**（宁可让人看到"没有 IMU"，也不能给假姿态）
  · 真实轮胎摩擦 —— 用一阶响应 + 有效轮距系数近似（交接包 F1 的实测口径）
"""

from __future__ import annotations

import math
import os
import struct
import sys
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from acg720_protocol_v1 import (  # noqa: E402
    FaultReason, Kind, MotionMode, PositionResult, PositionState, SafetyState,
    VERSION, crc16, encode_frame,
)

# ============================================================================
# 常量：全部从 RTL 折算出秒，行号即证据
# ============================================================================

#: 板载时钟（top.v:5,164 两处都写 50_000_000）
CLK_HZ = 50_000_000

#: 四轮索引顺序。top.v:4 注释 + chassis_telemetry_imu_tx.v:254 的拼接顺序共同确定：
#:     {rear_right, rear_left, front_left, front_right}
#: 于是 A=右前、B=左前、C=左后、D=右后。左侧 = B/C，右侧 = A/D。
WHEEL_A_RIGHT_FRONT = 0
WHEEL_B_LEFT_FRONT = 1
WHEEL_C_LEFT_REAR = 2
WHEEL_D_RIGHT_REAR = 3
RIGHT_WHEELS = (WHEEL_A_RIGHT_FRONT, WHEEL_D_RIGHT_REAR)
LEFT_WHEELS = (WHEEL_B_LEFT_FRONT, WHEEL_C_LEFT_REAR)


@dataclass(frozen=True)
class FirmwareParams:
    """固件与被控对象的全部参数。**默认值就是真值**。

    ⚠ 想"放宽 3 秒上限让测试好过"时请停下来想一下：那正是我们要验证的那条限制。
      测试要放宽必须**显式**传参（`FirmwareParams(run_limit_s=0.5)`），
      免得有人把真问题关掉还以为测试是绿的。
    """

    # ---- 串口 / 帧层（chassis_command_rx.v:11,45）----
    baudrate: int = 115200
    version: int = VERSION
    #: FRAME_TIMEOUT_CYCLES = 5_000_000 → 100 ms 帧内字节间隔上限
    frame_timeout_s: float = 5_000_000 / CLK_HZ
    #: LEN 上限（chassis_command_rx.v:208）。**不是** INTERFACE.md 里写的 48。
    max_command_payload: int = 8

    # ---- 安全状态机（chassis_safety_fsm.v:10,11）----
    #: COMMAND_TIMEOUT_CYCLES = 15_000_000 → 300 ms 无有效命令 → FAULT_TIMEOUT
    command_timeout_s: float = 15_000_000 / CLK_HZ
    #: RUN_LIMIT_CYCLES = 150_000_000 → 3.0 s 连续 RUNNING → FAULT_RUN_LIMIT
    run_limit_s: float = 150_000_000 / CLK_HZ

    # ---- 遥测 / 采样（chassis_telemetry_imu_tx.v:4、encoder_quadrature.v:12）----
    #: TELEMETRY_PERIOD_CYCLES = 2_500_000 → 20 Hz
    telemetry_period_s: float = 2_500_000 / CLK_HZ
    #: SAMPLE_CYCLES = 2_500_000 → 50 ms
    sample_tick_s: float = 2_500_000 / CLK_HZ
    #: 位流版本号（chassis_telemetry_imu_tx.v:40）
    build_id: int = 0x20260A02

    # ---- 位置环（chassis_position_controller.v:9-17）----
    pos_min_rpm: int = 8
    pos_slow_rpm: int = 15
    pos_ramp_step_rpm: int = 4
    pos_slow_zone_counts: int = 400
    pos_creep_zone_counts: int = 160
    pos_stop_tol_counts: int = 32
    pos_settle_ticks: int = 3
    pos_stall_ticks: int = 10
    #: TIMEOUT_TICKS = 240，每 tick 50 ms → 一次位置动作最多 12 s
    pos_timeout_ticks: int = 240

    # ---- 取值上下限（chassis_command_rx.v:97-112）----
    max_move_rpm: int = 200
    max_open_loop_duty: int = 1400
    position_rpm_min: int = 8
    position_rpm_max: int = 40
    position_min_counts: int = 32

    # ---- 被控对象（交接包 + encoder_quadrature.v:6）----
    counts_per_rev: float = 1320.0
    wheel_radius_m: float = 0.0325
    geometry_separation_m: float = 0.200
    #: 真值（车体真实）有效轮距倍率。1.0 = 无侧滑；>1 复刻 skid 的"转不够"。
    ground_truth_separation_scale: float = 1.0
    #: 一阶电机响应时间常数。**拍值**，真机待实测，只影响瞬态不影响安全逻辑。
    motor_time_constant_s: float = 0.12
    #: 低速滑行下限（交接包 F1 实测≈7 RPM）：非零目标低于它时，轮子实际以它滑行。
    #: 这是**真车缺陷**，必须复刻 —— 否则"让它慢挪它却冲出去"只会在车上发现。
    tracking_floor_rpm: float = 7.0
    #: False = 台架上没插 IMU。0x82 会照发，但 flags 报 stale。
    imu_available: bool = False

    # ---- 派生量 ----

    @property
    def wire_time_per_byte_s(self) -> float:
        """一个字节在线上占的时间：起始位 + 8 数据位 + 停止位 = 10 bit。"""
        return 10.0 / float(self.baudrate)

    @property
    def counts_per_metre(self) -> float:
        return self.counts_per_rev / (2.0 * math.pi * self.wheel_radius_m)

    @property
    def ground_truth_separation_m(self) -> float:
        return self.geometry_separation_m * self.ground_truth_separation_scale


# ============================================================================
# RX：帧状态机（chassis_command_rx.v 逐行复刻）
# ============================================================================

#: 接收侧事件名。测试直接断言这些字符串，避免"看计数器猜发生了什么"。
EV_ACCEPTED = "accepted"
EV_DUPLICATE = "duplicate"
EV_CRC_ERROR = "crc_error"
EV_VERSION_ERROR = "version_error"
EV_LENGTH_ERROR = "length_error"
EV_SEMANTIC_ERROR = "semantic_error"
EV_FRAME_TIMEOUT = "frame_timeout"
EV_UART_FRAME_ERROR = "uart_frame_error"


class RxFsm:
    """接收状态机。9 个状态，与 `chassis_command_rx.v:37-44` 一一对应。

    三个容易做错、而真车上代价极大的地方：

    1. **`ST_HEADER_1` 收到 `A5` 时留在原状态**（:181-183）。所以
       `A5 A5 A5 5A` 能自恢复。写成"不是 5A 就回 HEADER_0"会让这种流永久失步。
    2. **VERSION 错 / LEN>8 / CRC 错 / 语义不合法，四类都置 `protocol_fault`**
       （:191, :208, :245, :247）。固件不区分"格式错"和"内容错"——
       全都锁死底盘。**只有 100 ms 帧内超时例外**（:156-160）：它只加
       `frame_error_count`、回 HEADER_0，**不置 fault**。
    3. **SEQ 不新且不是 STOP → 只加 `duplicate_count`，不回 ACK**（:287-289）。
       现象是"命令发出去了、车不动、毫无报错"。
    """

    ST_HEADER_0 = 0
    ST_HEADER_1 = 1
    ST_VERSION = 2
    ST_KIND = 3
    ST_SEQ = 4
    ST_LENGTH = 5
    ST_PAYLOAD = 6
    ST_CRC_LOW = 7
    ST_CRC_HIGH = 8

    def __init__(self, params: FirmwareParams) -> None:
        self.p = params
        self.state = self.ST_HEADER_0
        self.frame_age_s = 0.0
        self.crc_running = 0xFFFF
        self.kind_hold = 0
        self.seq_hold = 0
        self.length_hold = 0
        self.payload = bytearray()
        self.payload_index = 0
        self.crc_low = 0

        # 计数器（chassis_command_rx.v:143-145），会出现在 0x81 遥测里
        self.crc_errors = 0
        self.frame_errors = 0
        self.duplicates = 0

        #: SEQ 追踪寄存器（:283-286）
        self.last_seq = 0
        self.last_seq_valid = False

        #: 接受一帧后置一次；由上层消费（等价于 `command_valid` 单周期脉冲）
        self.last_frame: Optional[dict] = None
        #: 本拍是否置 `protocol_fault`（单周期脉冲，由上层消费）
        self.protocol_fault = False

    # ---- 时间推进：帧内字节超时 ----

    def advance(self, dt: float) -> List[str]:
        """推进 `frame_age`。**这一步不置 fault**（:156-160）。"""
        if self.state == self.ST_HEADER_0:
            self.frame_age_s = 0.0
            return []
        self.frame_age_s += dt
        if self.frame_age_s >= self.p.frame_timeout_s:
            self.state = self.ST_HEADER_0
            self.frame_age_s = 0.0
            self._count('frame_errors')
            return [EV_FRAME_TIMEOUT]
        return []

    def inject_uart_frame_error(self) -> None:
        """模拟 UART 帧错误（停止位错/break）。真固件会置 `protocol_fault`（:165-170）。"""
        self.state = self.ST_HEADER_0
        self.frame_age_s = 0.0
        self.protocol_fault = True
        self._count('frame_errors')

    # ---- 逐字节 ----

    def feed_byte(self, byte: int) -> List[str]:
        events: List[str] = []
        if self.state == self.ST_HEADER_0:
            if byte == 0xA5:
                self.state = self.ST_HEADER_1

        elif self.state == self.ST_HEADER_1:
            if byte == 0x5A:
                self.state = self.ST_VERSION
                self.crc_running = 0xFFFF
            elif byte != 0xA5:
                self.state = self.ST_HEADER_0
            # byte == 0xA5：**留在 HEADER_1**（自恢复，:181-183）

        elif self.state == self.ST_VERSION:
            if byte == self.p.version:
                self.crc_running = _crc16_byte(self.crc_running, byte)
                self.state = self.ST_KIND
            else:
                self.state = self.ST_HEADER_0
                self.protocol_fault = True
                events.append(EV_VERSION_ERROR)

        elif self.state == self.ST_KIND:
            self.kind_hold = byte
            self.crc_running = _crc16_byte(self.crc_running, byte)
            self.state = self.ST_SEQ

        elif self.state == self.ST_SEQ:
            self.seq_hold = byte
            self.crc_running = _crc16_byte(self.crc_running, byte)
            self.state = self.ST_LENGTH

        elif self.state == self.ST_LENGTH:
            self.length_hold = byte
            self.crc_running = _crc16_byte(self.crc_running, byte)
            self.payload = bytearray()
            self.payload_index = 0
            if byte > self.p.max_command_payload:
                self.state = self.ST_HEADER_0
                self.protocol_fault = True
                events.append(EV_LENGTH_ERROR)
            elif byte == 0:
                self.state = self.ST_CRC_LOW
            else:
                self.state = self.ST_PAYLOAD

        elif self.state == self.ST_PAYLOAD:
            self.crc_running = _crc16_byte(self.crc_running, byte)
            self.payload.append(byte)
            if self.payload_index == self.length_hold - 1:
                self.state = self.ST_CRC_LOW
            else:
                self.payload_index += 1

        elif self.state == self.ST_CRC_LOW:
            self.crc_low = byte
            self.state = self.ST_CRC_HIGH

        elif self.state == self.ST_CRC_HIGH:
            self.state = self.ST_HEADER_0
            got = (byte << 8) | self.crc_low          # {rx_data, crc_low}（:119）
            if got != self.crc_running:
                self._count('crc_errors')
                self.protocol_fault = True
                events.append(EV_CRC_ERROR)
            else:
                events.extend(self._finish_frame())

        self.frame_age_s = 0.0
        return events

    # ---- 帧尾判定（:240-290）----

    def _finish_frame(self) -> List[str]:
        kind = self.kind_hold
        length = self.length_hold
        payload = bytes(self.payload)

        if not self.semantic_valid(kind, length, payload):
            self.protocol_fault = True
            return [EV_SEMANTIC_ERROR]

        sequence_new = self._sequence_new()
        if sequence_new or kind == int(Kind.STOP):
            self.last_frame = {
                'kind': kind,
                'sequence': self.seq_hold,
                'payload': payload,
                'sequence_new': sequence_new,
            }
            if sequence_new:
                self.last_seq = self.seq_hold
                self.last_seq_valid = True
            return [EV_ACCEPTED]
        self._count('duplicates')
        return [EV_DUPLICATE]

    def _sequence_new(self) -> bool:
        """`sequence_new = !last_seq_valid || (delta != 0 && !delta[7])`（:93-95）。

        即 delta 必须落在 **1..127**。delta==0（重复）与 delta>=128（落后）都被丢弃。
        """
        if not self.last_seq_valid:
            return True
        delta = (self.seq_hold - self.last_seq) & 0xFF
        return delta != 0 and delta < 128

    def reset_seq_tracking(self) -> None:
        """仅供主机侧测试复位用；真固件只有 `rst_n` 能清。"""
        self.last_seq = 0
        self.last_seq_valid = False

    def semantic_valid(self, kind: int, length: int, payload: bytes) -> bool:
        """`semantic_valid` 的逐条复刻（chassis_command_rx.v:97-112）。

        ⚠ POSITION 的转速上限检查用的是 **payload[1] 单字节**（`payload_rpm_low`），
          不是拼出来的 16 位值 —— 因为第 2 个字节在 POSITION 里属于计数。
        """
        p = self.p
        if len(payload) != length:
            return False

        def u16(lo_index: int) -> int:
            return payload[lo_index] | (payload[lo_index + 1] << 8)

        if length > p.max_command_payload:
            return False

        if kind == int(Kind.MOVE):
            return (length == 4 and payload[0] <= int(MotionMode.ROTATE_RIGHT)
                    and u16(1) <= p.max_move_rpm)

        if kind == int(Kind.OPEN_LOOP):
            return (length == 4
                    and payload[0] in (int(MotionMode.FORWARD), int(MotionMode.BACKWARD))
                    and u16(1) <= p.max_open_loop_duty
                    and payload[3] == 0)

        if kind == int(Kind.POSITION):
            return (length == 8
                    and payload[0] in (int(MotionMode.FORWARD), int(MotionMode.BACKWARD),
                                       int(MotionMode.ROTATE_LEFT), int(MotionMode.ROTATE_RIGHT))
                    and payload[1] >= p.position_rpm_min
                    and payload[1] <= p.position_rpm_max
                    and u32(payload, 2) > p.position_min_counts
                    and u16(6) == 0)

        if kind in (int(Kind.STOP), int(Kind.ARM), int(Kind.DISARM),
                    int(Kind.HEARTBEAT)):
            return length == 0

        return False

    def _count(self, name: str) -> None:
        """计数器是 16 位、饱和不回绕（:159-160, :243-244, :288）。"""
        value = getattr(self, name)
        if value != 0xFFFF:
            setattr(self, name, value + 1)


def _crc16_byte(crc: int, byte: int) -> int:
    """等价于固件的 `crc16_byte()`（chassis_command_rx.v:54-70），也与本模块的
    `crc16(单字节, crc)` 一致 —— 这里改用表驱动实现以保证与 `acg720_protocol_v1`
    的 CRC **逐位相同**，避免两处各写一遍多项式。
    """
    return crc16(bytes((byte,)), crc)


def u32(payload: bytes, offset: int) -> int:
    return struct.unpack_from('<I', payload, offset)[0]


def u16(payload: bytes, offset: int) -> int:
    return struct.unpack_from('<H', payload, offset)[0]


# ============================================================================
# 动作码 → 四轮转速（chassis_motion.v 逐行复刻）
# ============================================================================

def chassis_motion(mode: int, base_rpm: int, turn_inner_ratio_q8: int) -> Tuple[int, int, int, int]:
    """返回 `(A右前, B左前, C左后, D右后)` 的有符号目标 RPM。

    复刻 `chassis_motion.v:32-87`。**必须照抄的一点**：
    `inner_rpm = base_rpm * ratio >> 8` 是**无符号**的（:36），
    所以内侧轮**只能缩小、不能反转、不能独立赋值**。这直接决定了
    "倒车转弯"和"转弯半径 < 半轮距"这两类 (v, ω) 表达不出来。
    """
    if base_rpm > 200 or mode > int(MotionMode.ROTATE_RIGHT):
        return (0, 0, 0, 0)                       # target_valid = 0（:32-33）

    outer = base_rpm
    inner = (base_rpm * turn_inner_ratio_q8) >> 8     # :34,36

    if mode == int(MotionMode.STOP):
        return (0, 0, 0, 0)
    if mode == int(MotionMode.FORWARD):
        return (outer, outer, outer, outer)
    if mode == int(MotionMode.BACKWARD):
        return (-outer, -outer, -outer, -outer)
    if mode == int(MotionMode.TURN_LEFT):            # 左轮为内侧（:59-64）
        return (outer, inner, inner, outer)
    if mode == int(MotionMode.TURN_RIGHT):           # 右轮为内侧（:65-70）
        return (inner, outer, outer, inner)
    if mode == int(MotionMode.ROTATE_LEFT):          # （:71-76）
        return (outer, -outer, -outer, outer)
    if mode == int(MotionMode.ROTATE_RIGHT):         # （:77-82）
        return (-outer, outer, outer, -outer)
    return (0, 0, 0, 0)


# ============================================================================
# 位置环（chassis_position_controller.v 逐行复刻）
# ============================================================================

class PositionLoop:
    """0x05 POSITION 的闭环。**当前固件唯一不受 3 s 上限约束的运动方式。**

    反直觉但必须照抄的一条：`progress = (Σ 归一化增量) >> 2`（:78）
    —— 是四个轮子增量的**平均**，不是求和。所以 `target_counts` 的语义是
    "每个轮子滚过的计数"，不是"四个轮子加起来"。

    原地转的归一化（:126-133）把右前/右后取正、左前/左后取负，
    于是"左右各滚 x 个计数"在平均后仍然等于 x。两种动作的口径因此统一。
    """

    def __init__(self, params: FirmwareParams) -> None:
        self.p = params
        self.state = PositionState.IDLE
        self.result_code = PositionResult.NONE
        self.busy = False
        self.done = False
        self.motion_mode = 0
        self.base_rpm = 0
        self.target_counts_latched = 0
        self.progress_counts = 0
        self.remaining_counts = 0
        self.max_rpm_latched = 0
        self.internal_stop_pulse = False
        self.aborted_count = 0
        self.rejected_count = 0
        self._start = [0, 0, 0, 0]
        self._previous_progress = 0
        self._settle_counter = 0
        self._stall_counter = 0
        self._timeout_counter = 0

    # ---- 启动 ----

    def valid_start(self, mode: int, target_counts: int, max_rpm: int) -> bool:
        """`valid_start`（:86-87）：动作码 ∈ {1,2,5,6}、计数 > 32、8 ≤ rpm ≤ 40。"""
        return (mode in (int(MotionMode.FORWARD), int(MotionMode.BACKWARD),
                         int(MotionMode.ROTATE_LEFT), int(MotionMode.ROTATE_RIGHT))
                and target_counts > self.p.pos_stop_tol_counts
                and self.p.pos_min_rpm <= max_rpm <= self.p.position_rpm_max)

    def start(self, mode: int, target_counts: int, max_rpm: int,
              positions: List[int]) -> None:
        """`start_valid && !busy` 分支（:177-206）。

        ⚠ busy 期间**再发 POSITION 什么都不会发生**（既不重启也不中止）——
          因为 `position_start` 只看 kind，而 `position_abort` 只认
          STOP/MOVE/DISARM/OPEN_LOOP（top.v:121-125）。
        """
        if self.busy:
            return
        self.done = False
        self.progress_counts = 0
        self.remaining_counts = target_counts
        self._settle_counter = 0
        self._stall_counter = 0
        self._timeout_counter = 0
        self._previous_progress = 0
        if self.valid_start(mode, target_counts, max_rpm):
            self.motion_mode = mode
            self.base_rpm = self.p.pos_min_rpm
            self.busy = True
            self.state = PositionState.RUN
            self.result_code = PositionResult.NONE
            self.target_counts_latched = target_counts
            self.max_rpm_latched = max_rpm
            self._start = list(positions[:4])
        else:
            # RESULT_REJECTED（:197-206）：立刻 done，并且发一记 internal_stop
            self.motion_mode = 0
            self.base_rpm = 0
            self.busy = False
            self.done = True
            self.state = PositionState.ABORT
            self.result_code = PositionResult.REJECTED
            self.target_counts_latched = target_counts
            self.internal_stop_pulse = True
            self.rejected_count += 1

    def abort(self, positions: List[int]) -> None:
        """`abort && busy`（:169-176）→ RESULT_ABORTED。

        `abort` 的来源见 top.v:121-125：`!estop_n`、FAULT 状态、
        或收到 STOP/MOVE/DISARM/OPEN_LOOP。
        """
        if not self.busy:
            return
        self.motion_mode = 0
        self.base_rpm = 0
        self.busy = False
        self.done = False
        self.state = PositionState.ABORT
        self.result_code = PositionResult.ABORTED
        self.internal_stop_pulse = True
        self.aborted_count += 1

    # ---- 每个 50 ms tick（:207-266）----

    def step(self, positions: List[int], measured_rpm: List[int]) -> None:
        if not self.busy:
            return
        self._timeout_counter += 1

        progress = self._progress(positions)
        remaining = 0 if progress >= self.target_counts_latched \
            else self.target_counts_latched - progress
        self.progress_counts = progress
        self.remaining_counts = remaining

        if self.state == PositionState.RUN:
            if remaining <= self.p.pos_stop_tol_counts:
                self.base_rpm = 0
                self.state = PositionState.SETTLE
                self._settle_counter = 0
                self._stall_counter = 0
            elif self._timeout_counter >= self.p.pos_timeout_ticks:
                self._finish(PositionResult.TIMEOUT)
            else:
                if progress <= self._previous_progress + 1:
                    self._stall_counter += 1
                else:
                    self._stall_counter = 0
                if self._stall_counter >= self.p.pos_stall_ticks:
                    self._finish(PositionResult.STALL)
                else:
                    self.base_rpm = self._ramp(self.base_rpm, self._desired_rpm(remaining))
                    self._previous_progress = progress

        elif self.state == PositionState.SETTLE:
            self.base_rpm = 0
            if all(v == 0 for v in measured_rpm[:4]):
                if self._settle_counter >= self.p.pos_settle_ticks - 1:
                    self.motion_mode = 0
                    self.busy = False
                    self.done = True
                    self.state = PositionState.DONE
                    self.result_code = PositionResult.OK
                    self.internal_stop_pulse = True
                else:
                    self._settle_counter += 1
            else:
                self._settle_counter = 0

    def _finish(self, result: PositionResult) -> None:
        self.motion_mode = 0
        self.base_rpm = 0
        self.busy = False
        self.state = PositionState.ABORT
        self.result_code = result
        self.internal_stop_pulse = True

    def _desired_rpm(self, remaining: int) -> int:
        """转速剖面（:93-102）：越接近目标越慢，最后 160 计数用 MIN_RPM 蠕行。"""
        if remaining <= self.p.pos_stop_tol_counts:
            return 0
        if remaining <= self.p.pos_creep_zone_counts:
            return self.p.pos_min_rpm
        if remaining <= self.p.pos_slow_zone_counts:
            return min(self.max_rpm_latched, self.p.pos_slow_rpm)
        return self.max_rpm_latched

    def _ramp(self, current: int, desired: int) -> int:
        """每 tick 最多变 4 RPM（:240-246）。"""
        step = self.p.pos_ramp_step_rpm
        if desired > current + step:
            return current + step
        if current > desired + step:
            return current - step
        return desired

    def _progress(self, positions: List[int]) -> int:
        """`(Σ 归一化增量) >> 2`，算术右移、负数截 0（:78-79, :117-141）。"""
        deltas = [positions[i] - self._start[i] for i in range(4)]
        mode = self.motion_mode
        if mode == int(MotionMode.FORWARD):
            norm = deltas
        elif mode == int(MotionMode.BACKWARD):
            norm = [-d for d in deltas]
        elif mode == int(MotionMode.ROTATE_LEFT):
            # A、D 取正，B、C 取负（:126-129）
            norm = [deltas[0], -deltas[1], -deltas[2], deltas[3]]
        elif mode == int(MotionMode.ROTATE_RIGHT):
            norm = [-deltas[0], deltas[1], deltas[2], -deltas[3]]
        else:
            norm = [0, 0, 0, 0]
        total = sum(norm)
        if total < 0:
            total = -((-total) >> 2)          # 等价于算术右移（向负无穷取整）
        else:
            total = total >> 2
        return 0 if total < 0 else total


# ============================================================================
# 安全状态机（chassis_safety_fsm.v 逐行复刻）
# ============================================================================

class SafetyFsm:
    """ARM / 急停 / 超时 / 运行上限 / 故障锁存。

    三条**必须照抄**的行为，错一条就会在真车上表现为"莫名其妙不动了"：

    1. **FAULT 自锁**（:88）：`else if (state == STATE_FAULT) state <= STATE_FAULT;`
       没有任何命令能清除它。只能 `reset_n`，或"松开 S0 再按 S4"
       （`estop_seen` 的清零条件，:69-72）。
    2. **`run_age` 只在离开 RUNNING 时清零**（:72-77）。重发命令**不清零**，
       所以"高频重发 MOVE 就能一直跑"是错的 —— 3.0 s 必进 FAULT_RUN_LIMIT。
    3. **DISARMED 下非 ARM 命令被静默忽略**（:120-126 的 case 只匹配 CMD_ARM），
       不报错、不置 fault。现象是"发了没反应"。
    """

    def __init__(self, params: FirmwareParams) -> None:
        self.p = params
        self.state = SafetyState.DISARMED
        self.fault_reason = FaultReason.NONE
        self.command_age_s = 0.0
        self.run_age_s = 0.0
        self.estop_seen = False
        self.reset_n = True
        self.estop_n = True
        self.motor_fault = False
        self.protocol_fault = False
        self.internal_stop = False
        self.position_active = False

    # ---- 组合输出（:55-63）----

    @property
    def timeout_reached(self) -> bool:
        return self.command_age_s >= self.p.command_timeout_s

    @property
    def run_limit_reached(self) -> bool:
        return (not self.position_active) and self.run_age_s >= self.p.run_limit_s

    @property
    def armed(self) -> bool:
        return self.state in (SafetyState.ARMED_STOP, SafetyState.RUNNING)

    @property
    def drive_allowed(self) -> bool:
        return (self.reset_n and self.estop_n and not self.estop_seen
                and self.state == SafetyState.RUNNING
                and not self.motor_fault and not self.protocol_fault
                and not self.timeout_reached)

    @property
    def motor_stby(self) -> bool:
        return self.drive_allowed and self.reset_n and self.estop_n and not self.motor_fault

    # ---- 外部输入 ----

    def set_estop(self, released: bool) -> None:
        """`estop_n` 引脚。拉低会**异步**置 `estop_seen`（:69-72）。"""
        self.estop_n = released
        if not released:
            self.estop_seen = True

    def set_reset(self, released: bool) -> None:
        """`reset_n` 引脚 —— 也就是"按住 S4"。

        ★ 这是**唯一**能清除已锁存 FAULT 的手段（:88 的自锁）。
          注意 `estop_seen` 的清零条件是"`estop_n` 为高 **且** `reset_n` 为低"
          （:69-72）：必须先松开 S0，再按 S4。光松开急停是回不来的。
        """
        self.reset_n = released

    def inject_motor_fault(self, active: bool = True) -> None:
        """验证 FAULT_MOTOR 分支用。真故障来自 chassis_four_motor_control，本模型不模拟。"""
        self.motor_fault = active

    # ---- 一拍推进 ----

    def advance(self, dt: float, command: Optional[dict],
                internal_stop: bool) -> None:
        """推进一拍。

        `command` 是**本拍**有效的 `command_valid`（None = 本拍没有新命令）。
        `internal_stop` 是位置环的 `internal_stop_pulse`（top.v:236）。
        两个都要在**同一拍**可见，因为 RTL 里它们就是同一周期的信号。
        """
        if command is not None:
            self.command_age_s = 0.0
        else:
            self.command_age_s += dt

        if self.state != SafetyState.RUNNING:
            self.run_age_s = 0.0
        elif not self.run_limit_reached:
            self.run_age_s += dt

        # ---- 状态迁移（:74-175），顺序必须与 RTL 一致 ----
        if not self.reset_n:
            # S4 按住：`estop_seen` 清零（:71）且状态回 DISARMED（:75-79）
            self.estop_seen = False
            self.state = SafetyState.DISARMED
            self.fault_reason = FaultReason.NONE
            self.command_age_s = 0.0
            return

        if not self.estop_n or self.estop_seen:
            self.state = SafetyState.FAULT
            self.fault_reason = FaultReason.ESTOP
            self.command_age_s = 0.0
            return

        if self.state == SafetyState.FAULT:
            self.state = SafetyState.FAULT          # 自锁（:88）
            return

        if self.state == SafetyState.RUNNING and internal_stop:
            self.state = SafetyState.ARMED_STOP
            self.fault_reason = FaultReason.NONE
            self.command_age_s = 0.0
            return

        if self.state == SafetyState.RUNNING and self.run_limit_reached:
            self.state = SafetyState.FAULT
            self.fault_reason = FaultReason.RUN_LIMIT
            return

        if self.motor_fault:
            self.state = SafetyState.FAULT
            self.fault_reason = FaultReason.MOTOR
            return

        if self.protocol_fault:
            self.state = SafetyState.FAULT
            self.fault_reason = FaultReason.PROTOCOL
            return

        if self.armed and self.timeout_reached and command is None:
            self.state = SafetyState.FAULT
            self.fault_reason = FaultReason.TIMEOUT
            return

        if command is None:
            return

        if not command['target_valid']:
            self.state = SafetyState.FAULT
            self.fault_reason = FaultReason.PROTOCOL
            return

        kind = command['kind']
        stop_target = command['stop_target']

        if self.state == SafetyState.DISARMED:
            if kind == int(Kind.ARM) and stop_target:
                self.state = SafetyState.ARMED_STOP

        elif self.state == SafetyState.ARMED_STOP:
            if kind == int(Kind.STOP):
                self.state = SafetyState.ARMED_STOP
            elif kind == int(Kind.ARM):
                if not stop_target:
                    # ARMED_STOP 下再发带目标的 ARM → FAULT（:139-146）
                    self.state = SafetyState.FAULT
                    self.fault_reason = FaultReason.PROTOCOL
            elif kind in (int(Kind.MOVE), int(Kind.OPEN_LOOP), int(Kind.POSITION)):
                if not stop_target:
                    self.state = SafetyState.RUNNING
            elif kind == int(Kind.HEARTBEAT):
                self.state = SafetyState.ARMED_STOP
            elif kind == int(Kind.DISARM):
                self.state = SafetyState.DISARMED

        elif self.state == SafetyState.RUNNING:
            if kind == int(Kind.STOP):
                self.state = SafetyState.ARMED_STOP
            elif kind in (int(Kind.MOVE), int(Kind.OPEN_LOOP), int(Kind.POSITION)):
                if stop_target:
                    self.state = SafetyState.ARMED_STOP
            elif kind == int(Kind.HEARTBEAT):
                self.state = SafetyState.RUNNING
            elif kind == int(Kind.DISARM):
                self.state = SafetyState.DISARMED


# ============================================================================
# 被控对象：电机 + 编码器 + 车体位姿
# ============================================================================

class ChassisPlant:
    """四个轮子 + 编码器 + 车体真值位姿。

    ⚠ 真值位姿**只存在于模型里**，真机没有。它的用途是给标定提供参照
      （"命令转 1 圈、车实际只转 0.73 圈"这个 skid 现象必须在虚拟车上能看到），
      所以它用 `ground_truth_separation_m`（带侧滑系数），而不是几何轮距。
    """

    def __init__(self, params: FirmwareParams) -> None:
        self.p = params
        #: 浮点累计计数（真值）。上报时取整成 s32。
        self.count_f = [0.0, 0.0, 0.0, 0.0]
        self.position_counts = [0, 0, 0, 0]
        self.sample_deltas = [0, 0, 0, 0]
        self.measured_rpm = [0, 0, 0, 0]
        self.rpm_f = [0.0, 0.0, 0.0, 0.0]
        self.duty_cycles = [0, 0, 0, 0]
        self.channel_fault = [0, 0, 0, 0]
        self.invalid_transitions = [0, 0, 0, 0]
        self._last_sampled = [0, 0, 0, 0]
        # 车体真值
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        #: 进入滑行下限的采样数（诊断用）
        self.floor_sliding_samples = 0

    # ---- 每拍 ----

    def advance(self, dt: float, target_rpm: List[int], stby: bool) -> None:
        alpha = 1.0 - math.exp(-dt / self.p.motor_time_constant_s) \
            if self.p.motor_time_constant_s > 0 else 1.0

        targets = list(target_rpm[:4]) if stby else [0, 0, 0, 0]
        for i in range(4):
            target = float(targets[i])
            # ★ 低速滑行下限（交接包 F1 实测≈7 RPM）：这是**真车缺陷**，照抄
            if 1e-9 < abs(target) < self.p.tracking_floor_rpm:
                effective = math.copysign(self.p.tracking_floor_rpm, target)
                self.floor_sliding_samples += 1
            else:
                effective = target
            self.rpm_f[i] += (effective - self.rpm_f[i]) * alpha
            if abs(self.rpm_f[i]) < 1e-6:
                self.rpm_f[i] = 0.0
            self.count_f[i] += self.rpm_f[i] * self.p.counts_per_rev / 60.0 * dt
            # duty 只是让 0x81 遥测看着合理；不参与任何安全判定
            self.duty_cycles[i] = int(min(2700, max(0, 150 + abs(self.rpm_f[i]) * 12.0)))

        # 车体真值：用有效轮距（含侧滑系数）
        left = (self.rpm_f[WHEEL_B_LEFT_FRONT] + self.rpm_f[WHEEL_C_LEFT_REAR]) / 2.0
        right = (self.rpm_f[WHEEL_A_RIGHT_FRONT] + self.rpm_f[WHEEL_D_RIGHT_REAR]) / 2.0
        mps = 2.0 * math.pi * self.p.wheel_radius_m / 60.0
        v = (left + right) / 2.0 * mps
        omega = (right - left) * mps / self.p.ground_truth_separation_m
        yaw_mid = self.yaw + omega * dt / 2.0
        self.x += v * math.cos(yaw_mid) * dt
        self.y += v * math.sin(yaw_mid) * dt
        self.yaw = (self.yaw + omega * dt + math.pi) % (2.0 * math.pi) - math.pi

    def sample(self) -> None:
        """50 ms 编码器采样（encoder_quadrature.v:130-150）。"""
        for i in range(4):
            self.position_counts[i] = _to_s32(int(self.count_f[i]))
            delta = self.position_counts[i] - self._last_sampled[i]
            self._last_sampled[i] = self.position_counts[i]
            self.sample_deltas[i] = _to_s16(delta)
            self.measured_rpm[i] = _counts_to_rpm(delta)


def _to_s32(value: int) -> int:
    return ((value + 0x80000000) & 0xFFFFFFFF) - 0x80000000


def _to_s16(value: int) -> int:
    return ((value + 0x8000) & 0xFFFF) - 0x8000


def _counts_to_rpm(delta_counts: int) -> int:
    """`encoder_quadrature.v:53,105-129` 的定点换算。

    注释写明 `floor(count*10/11)` ≡ `(count*238313) >> 18`（0..32767 范围内），
    且 `|count|` 先被**饱和**到 32767。这里逐条照抄，
    所以 "RPM = 计数×10/11" 只是近似说法 —— 实际是定点乘加。
    """
    magnitude = min(abs(delta_counts), 32767)
    rpm = (magnitude * 238313) >> 18
    return -rpm if delta_counts < 0 else rpm


# ============================================================================
# 遥测发送：单槽 ACK + 单包在途 + tx_overflow（chassis_telemetry_imu_tx.v）
# ============================================================================

#: TX 仲裁优先级：ACK 排第一（:241-297 的 if/else 链顺序）
_TX_ORDER = ('ack', 'chassis', 'imu', 'diag', 'position')


#: TX 仲裁的子步长。RTL 每个时钟周期仲裁一次；这里用 0.5 ms 逼近。
#: 不这么做的话，`advance(0.02)` 一次只起得来一个包，一批四条遥测就要花 6 个 tick
#: （120 ms）才发完，遥测速率会莫名其妙地掉到 5 Hz —— 那是模型的假象，不是固件行为。
TX_SUBSTEP_S = 5e-4


class TxMux:
    """发送仲裁。

    ★ 会咬人的地方（chassis_telemetry_imu_tx.v:236-238）：
      `ack_pending` **只有一个槽**。若一条已被接受的命令的 ACK 还排在队里
      （最坏要等 46 字节的 0x81 帧发完，约 4 ms；四包齐发时约 14 ms），
      此时又来一条被接受的命令 → `tx_overflow <= 1'b1`，
      而 top.v:231 把它 OR 进了 `protocol_fault` → **永久锁死底盘**。

      所以主机**必须限速**发命令。这也是为什么本模型要按波特率推进时间：
      不推的话这个撞车永远复现不出来。

      反过来也要说清楚，免得把限制想得过紧：**两帧紧挨着的命令不会溢出**。
      ACK 帧只有 10 字节（0.87 ms），槽位在请求后第二拍就腾空了；
      要溢出必须"第二条命令落在**一条在途周期遥测**的窗口里"。
      20 Hz 遥测每 50 ms 一次、整批 164 字节约 14 ms，所以，
      只要命令周期明显大于 ~15 ms（即 ≲60 Hz），就不会撞上。
    """

    def __init__(self, params: FirmwareParams) -> None:
        self.p = params
        self.pending: Dict[str, bool] = {k: False for k in _TX_ORDER}
        self.ack_pending_seq = 0
        self.ack_seq_in = 0
        self.packet_active = False
        self.packet_kind: Optional[str] = None
        self.remaining_s = 0.0
        self.ack_request = False
        self.tx_overflow = False
        self.sent_bytes = bytearray()
        self.sent_count: Dict[str, int] = {k: 0 for k in _TX_ORDER}

    # ---- 请求 ----

    def request_ack(self, sequence: int) -> None:
        """外部请求发一帧 ACK。等价于 `ack_request` 的单周期脉冲。"""
        self.ack_seq_in = sequence
        self.ack_request = True

    def mark_periodic(self) -> None:
        """20 Hz 到点：四种周期遥测同时置 pending（:230-234）。"""
        for key in ('chassis', 'imu', 'diag', 'position'):
            self.pending[key] = True

    # ---- 推进 ----

    def advance(self, dt: float, build: Callable[[str, int], bytes]) -> None:
        """`build(kind, sequence)` 负责给出该包的**完整帧字节**（含帧头与 CRC）。

        顺序照抄 RTL：
          ① 先采样 `ack_request`：单槽已占 → `tx_overflow`；否则占槽
          ② 若没有在途包，按优先级取一个并开始发送（ACK 优先）

        与 RTL 的一处**刻意的**差异：RTL 里"置 pending"和"开始发送"差一个时钟周期
        （:236-247 是非阻塞赋值）。本模型用 `_advance(dt)` 推进，dt 可能是 20 ms，
        照抄那个"差一拍"会把 ACK 无谓地推迟一整个 tick。
        而"一个时钟周期内出现两次 `ack_request`"在物理上不可能
        （两帧之间至少隔 0.7 ms 的 UART 传输），所以这里允许同拍起发。
        真正要复刻的是"**槽被占住期间又来一条**" —— 那一条原样保留。
        """
        overflow = False
        request = self.ack_request
        self.ack_request = False

        left = dt
        first = True
        while first or left > 1e-12:
            step = min(left, TX_SUBSTEP_S)
            if first and request:
                if self.pending['ack']:
                    overflow = True               # :237
                else:
                    self.pending['ack'] = True
                    self.ack_pending_seq = self.ack_seq_in

            if self.packet_active:
                self.remaining_s -= step
                if self.remaining_s <= 0.0:
                    self.packet_active = False
                    self.packet_kind = None
                    self.remaining_s = 0.0
            else:
                for key in _TX_ORDER:
                    if not self.pending[key]:
                        continue
                    sequence = self.ack_pending_seq if key == 'ack' else 0
                    frame = build(key, sequence)
                    self.sent_bytes.extend(frame)
                    self.sent_count[key] += 1
                    self.packet_active = True
                    self.packet_kind = key
                    self.remaining_s = len(frame) * self.p.wire_time_per_byte_s
                    self.pending[key] = False
                    break

            first = False
            left -= step

        if overflow:
            self.tx_overflow = True

    def drain(self) -> bytes:
        out = bytes(self.sent_bytes)
        self.sent_bytes.clear()
        return out


# ============================================================================
# 固件总装
# ============================================================================

class Acg720Firmware:
    """把上面各部件按 `top.v` 的互连关系接起来。

    对外只有两个入口：
        `feed(data)`  喂串口收到的字节
        `tick(dt)`    推进 dt 秒，返回**本该发出去的字节**
    外加只读状态（`state` / `fault` / `build_id` / 各计数器）供测试与遥测使用。
    """

    def __init__(self, params: Optional[FirmwareParams] = None) -> None:
        self.p = params or FirmwareParams()
        self.rx = RxFsm(self.p)
        self.safety = SafetyFsm(self.p)
        self.position = PositionLoop(self.p)
        self.plant = ChassisPlant(self.p)
        self.tx = TxMux(self.p)

        # top.v:238-281 的 active_* 寄存器（手动 MOVE / OPEN_LOOP 的目标）
        self.active_motion_mode = 0
        self.active_base_rpm = 0
        self.active_turn_ratio = 0
        self.active_open_loop = False
        self.active_open_loop_duty = 0

        # 命令寄存器（chassis_command_rx.v 输出）
        self.command: Optional[dict] = None

        self.t = 0.0
        self.uptime_ms = 0
        self.diagnostic_counter = 0
        self.imu_sample_counter = 0

        self._sample_accum = 0.0
        self._telemetry_accum = 0.0
        self._pending_command: Optional[dict] = None
        self._protocol_fault_pulse = False
        self._protocol_fault_count = 0
        self._tx_overflow_count = 0
        self._last_frame_seq = 0
        #: 事件日志，测试与日志用（不是必需状态）
        self.events: List[str] = []

    # ==================================================================
    # 只读状态
    # ==================================================================

    @property
    def state(self) -> SafetyState:
        return self.safety.state

    @property
    def fault(self) -> FaultReason:
        return self.safety.fault_reason

    @property
    def build_id(self) -> int:
        return self.p.build_id

    @property
    def latched(self) -> bool:
        """是否已经锁死在 FAULT（只有硬件复位能出来）。"""
        return self.safety.state == SafetyState.FAULT

    def reset(self) -> None:
        """`rst_n`。真机上只有复位电路/重新上电会做这件事。"""
        self.__init__(self.p)

    # ==================================================================
    # 入口
    # ==================================================================

    def feed(self, data: bytes) -> List[str]:
        """喂串口字节。**每个字节按波特率推进模型时间**（真串口就是这样）。

        返回本批字节产生的接收事件名（`accepted` / `crc_error` / …），给测试和日志用。
        **要发出的字节用 `drain_tx()` 取** —— 一条命令的 ACK 通常要等到下一拍
        （真串口上一帧 0.7~4 ms 才发得完），所以 feed 之后立刻 drain 是取不到 ACK 的。
        """
        out: List[str] = []
        for byte in data:
            self._advance(self.p.wire_time_per_byte_s)
            for event in self.rx.feed_byte(byte):
                out.append(event)
                self._on_rx_event(event)
            command = self._take_frame()
            if command is not None:
                self._accept(command)
        return out

    def drain_tx(self) -> bytes:
        """取走当前已"发到线上"的字节。"""
        return self._collect()

    def tick(self, dt: float) -> bytes:
        """推进 dt 秒，返回这段时间里**应当发出**的字节。"""
        self._advance(dt)
        return self._collect()

    # ==================================================================
    # 内部：一拍
    # ==================================================================

    def _collect(self) -> bytes:
        """取走本拍该发出去的字节。"""
        return self.tx.drain()

    def _advance(self, dt: float) -> None:
        """一拍。次序与 RTL 的时钟沿一致，顺序本身有意义，改动前先想清楚：

            ① 时间                      ② RX 帧内超时（不置 fault）
            ③ uptime_ms                 ④ 20 Hz 遥测计数器 → pending
            ⑤ TX 仲裁 + 故障脉冲汇总      ⑥ 50 ms 采样：编码器 + 位置环
            ⑦ 安全状态机                 ⑧ 被控对象
        """
        self.t += dt

        # ② RX 帧内超时
        for event in self.rx.advance(dt):
            self._on_rx_event(event)

        # ③ 运行时间
        self.uptime_ms = int(self.t * 1000.0) & 0xFFFFFFFF

        # ④ 20 Hz 周期遥测
        self._telemetry_accum += dt
        while self._telemetry_accum >= self.p.telemetry_period_s:
            self._telemetry_accum -= self.p.telemetry_period_s
            self.tx.mark_periodic()
            self.diagnostic_counter = (self.diagnostic_counter + 1) & 0xFFFF
            self.imu_sample_counter = (self.imu_sample_counter + 1) & 0xFFFF

        # ⑤ TX 仲裁
        self.tx.advance(dt, self._build_packet)

        # ⑤b 汇总本拍的 `protocol_fault` 脉冲。
        #     top.v:231 把 tx_overflow 也 OR 进来，所以这里两路合并。
        #     合并后**不立刻清零**：⑦ 的安全状态机要在同一拍看到它。
        if self.rx.protocol_fault:
            self.rx.protocol_fault = False
            self._protocol_fault_pulse = True
        if self.tx.tx_overflow:
            self.tx.tx_overflow = False
            self._tx_overflow_count += 1
            self._protocol_fault_pulse = True
        if self._protocol_fault_pulse:
            self._protocol_fault_count += 1

        # ⑥ 50 ms 采样
        self._sample_accum += dt
        while self._sample_accum >= self.p.sample_tick_s:
            self._sample_accum -= self.p.sample_tick_s
            self._on_sample_tick()

        # ⑦ 安全状态机。command_valid / protocol_fault / internal_stop
        #    在 RTL 里都是**同一周期**的信号，所以这里也必须同拍可见。
        self.safety.protocol_fault = self._protocol_fault_pulse
        self.safety.internal_stop = self.position.internal_stop_pulse
        self.safety.position_active = self.position.busy
        self.safety.advance(dt, self._pending_command, self.position.internal_stop_pulse)
        self._pending_command = None
        self._protocol_fault_pulse = False
        self.position.internal_stop_pulse = False

        # ⑧ 被控对象
        self.plant.advance(dt, self.target_rpm(), self.safety.motor_stby)

    def _on_sample_tick(self) -> None:
        self.plant.sample()
        self.position.step(self.plant.position_counts, self.plant.measured_rpm)

    def _on_rx_event(self, event: str) -> None:
        self.events.append(event)
        if len(self.events) > 256:
            del self.events[:128]

    # ==================================================================
    # 命令接收 → 寄存器（chassis_command_rx.v 输出 + top.v 互连）
    # ==================================================================

    def _take_frame(self) -> Optional[dict]:
        """把 RX 接受的一帧翻译成 `command_*` 寄存器（:250-282）。"""
        frame = self.rx.last_frame
        if frame is None:
            return None
        self.rx.last_frame = None
        kind = frame['kind']
        payload = frame['payload']

        mode = payload[0] if payload else 0
        rpm16 = u16(payload, 1) if len(payload) >= 3 else 0
        ratio = payload[3] if len(payload) >= 4 else 0
        counts = u32(payload, 2) if len(payload) >= 6 else 0
        reserved = u16(payload, 6) if len(payload) >= 8 else 0

        if kind == int(Kind.MOVE):
            motion_mode, base_rpm = mode & 0x07, rpm16
            duty, position_counts, position_max_rpm = 0, 0, 0
            turn_ratio = ratio
        elif kind == int(Kind.OPEN_LOOP):
            motion_mode, base_rpm = mode & 0x07, 0
            duty, turn_ratio = rpm16, 0
            position_counts, position_max_rpm = 0, 0
        elif kind == int(Kind.POSITION):
            motion_mode, base_rpm, duty, turn_ratio = mode & 0x07, 0, 0, 0
            position_counts, position_max_rpm = counts, payload[1]
        else:
            motion_mode = base_rpm = duty = turn_ratio = 0
            position_counts = position_max_rpm = 0

        cmd = {
            'kind': kind,
            'sequence': frame['sequence'],
            'sequence_new': frame['sequence_new'],
            'motion_mode': motion_mode,
            'base_rpm': base_rpm,
            'turn_inner_ratio_q8': turn_ratio,
            'open_loop_duty': duty,
            'position_counts': position_counts,
            'position_max_rpm': position_max_rpm,
            # top.v:126-147 的两个组合判定
            'stop_target': _command_stop_target(
                kind, duty, position_counts, motion_mode, base_rpm),
            'target_valid': _command_target_valid(
                kind, duty, position_counts, position_max_rpm, motion_mode, base_rpm,
                self.p),
        }
        return cmd

    def _accept(self, cmd: dict) -> None:
        """接受一条命令后，RTL 里同一拍发生的所有副作用。"""
        self.command = cmd
        self._pending_command = cmd
        self._last_frame_seq = cmd['sequence']
        self.tx.request_ack(cmd['sequence'])

        # ---- top.v:238-281 的 active_* ----
        if (cmd['kind'] == int(Kind.MOVE) and cmd['motion_mode'] != 0
                and cmd['base_rpm'] != 0):
            self.active_motion_mode = cmd['motion_mode']
            self.active_base_rpm = cmd['base_rpm']
            self.active_turn_ratio = cmd['turn_inner_ratio_q8']
            self.active_open_loop = False
            self.active_open_loop_duty = 0
        elif cmd['kind'] == int(Kind.OPEN_LOOP) and cmd['open_loop_duty'] != 0:
            self.active_motion_mode = cmd['motion_mode']
            self.active_base_rpm = 1                 # top.v:265 写死 1
            self.active_turn_ratio = 0
            self.active_open_loop = True
            self.active_open_loop_duty = cmd['open_loop_duty']
        else:
            # ★ HEARTBEAT 也会走到这里，把手动目标清零（top.v:273-279）。
            #   所以"运动期间用 100 ms 心跳喂狗"在真机上会把车刹停。
            self.active_motion_mode = 0
            self.active_base_rpm = 0
            self.active_turn_ratio = 0
            self.active_open_loop = False
            self.active_open_loop_duty = 0

        # ---- position controller（top.v:120-125, 303-310）----
        if cmd['kind'] == int(Kind.POSITION):
            self.position.start(cmd['motion_mode'], cmd['position_counts'],
                                cmd['position_max_rpm'], self.plant.position_counts)
        if _position_abort(cmd, self.p):
            self.position.abort(self.plant.position_counts)

    # ==================================================================
    # 目标转速（top.v:148-152, 283-301）
    # ==================================================================

    def target_rpm(self) -> List[int]:
        """`target_rpm = position_busy ? position_target_rpm : manual_target_rpm`。"""
        if self.position.busy:
            return list(chassis_motion(self.position.motion_mode,
                                       self.position.base_rpm, 0))
        if self.safety.state == SafetyState.FAULT or self.safety.estop_seen:
            return [0, 0, 0, 0]
        return list(chassis_motion(self.active_motion_mode,
                                   self.active_base_rpm, self.active_turn_ratio))

    # ==================================================================
    # 遥测组帧（chassis_telemetry_imu_tx.v 的 packet_byte 逐字节对应）
    # ==================================================================

    def _build_packet(self, key: str, sequence: int) -> bytes:
        if key == 'ack':
            return encode_frame(Kind.ACK, self.tx.ack_pending_seq,
                                bytes((int(self.state), int(self.fault))))
        if key == 'chassis':
            return encode_frame(Kind.TELEMETRY, self._last_frame_seq,
                                self._chassis_payload())
        if key == 'imu':
            return encode_frame(Kind.IMU, self.imu_sample_counter & 0xFF,
                                self._imu_payload())
        if key == 'diag':
            return encode_frame(Kind.DIAGNOSTIC, self.diagnostic_counter & 0xFF,
                                self._diagnostic_payload())
        if key == 'position':
            return encode_frame(Kind.POSITION_TELEMETRY, self.diagnostic_counter & 0xFF,
                                self._position_payload())
        raise ValueError(f"未知包类型 {key!r}")

    def _chassis_payload(self) -> bytes:
        """0x81，38 字节。布局见 `acg720_protocol_v1.parse_telemetry`。"""
        body = bytearray()
        body.append(int(self.state))
        body.append(int(self.fault))
        body.append(self._last_frame_seq & 0xFF)
        body.append(1 if self.safety.motor_stby else 0)
        body += struct.pack('<4h', *self.target_rpm())
        body += struct.pack('<4h', *[_to_s16(v) for v in self.plant.measured_rpm])
        body += struct.pack('<4H', *[v & 0xFFFF for v in self.plant.duty_cycles])
        body += bytes(v & 0xFF for v in self.plant.channel_fault)
        body += struct.pack('<3H', self.rx.crc_errors & 0xFFFF,
                            self.rx.duplicates & 0xFFFF,
                            self.rx.frame_errors & 0xFFFF)
        assert len(body) == 38, len(body)
        return bytes(body)

    def _imu_payload(self) -> bytes:
        """0x82，36 字节。

        ⚠ 没有 IMU 就报 stale —— **绝不伪造有效数据**。
           flags 位序见 chassis_telemetry_imu_tx.v:264-266（MSB 先拼接）。
        """
        valid = 0
        if self.p.imu_available:
            valid = 0x07
        flags = ((1 if not self.p.imu_available else 0) << 7) | (0 << 6) | (0 << 5) \
            | (0 if self.p.imu_available else 1) << 4 \
            | (1 if self.p.imu_available else 0) << 3 | valid
        body = bytearray()
        body.append(flags)
        body.append(0)
        body += struct.pack('<H', self.imu_sample_counter & 0xFFFF)
        body += struct.pack('<9h', *([0] * 9))
        body += struct.pack('<h', 0)
        body += struct.pack('<H', 0xFFFF if not self.p.imu_available else 0)
        body += struct.pack('<3H', 0, 0, 0)
        body += struct.pack('<I', self.uptime_ms)
        assert len(body) == 36, len(body)
        return bytes(body)

    def _diagnostic_payload(self) -> bytes:
        """0x83，42 字节。build_id 在偏移 6 —— 版本检测的抓手。"""
        body = bytearray()
        body += struct.pack('<H', self.diagnostic_counter & 0xFFFF)
        body += struct.pack('<I', self.uptime_ms)
        body += struct.pack('<I', self.p.build_id)
        body += struct.pack('<4i', *self.plant.position_counts)
        body += struct.pack('<4h', *self.plant.sample_deltas)
        body += struct.pack('<4H', *[v & 0xFFFF for v in self.plant.invalid_transitions])
        assert len(body) == 42, len(body)
        return bytes(body)

    def _position_payload(self) -> bytes:
        """0x84，16 字节。flags = {result[3:0], done, busy}（:291）。"""
        flags = ((int(self.position.result_code) & 0x0F) << 2) \
            | (0x02 if self.position.done else 0) \
            | (0x01 if self.position.busy else 0)
        meta = (self.position.state & 0x07) \
            | ((self.position.motion_mode & 0x07) << 3)
        body = bytearray([flags, meta])
        body += struct.pack('<H', self.position.base_rpm & 0xFFFF)
        body += struct.pack('<I', self.position.target_counts_latched & 0xFFFFFFFF)
        body += struct.pack('<i', self.position.progress_counts)
        body += struct.pack('<I', self.position.remaining_counts & 0xFFFFFFFF)
        assert len(body) == 16, len(body)
        return bytes(body)


# ============================================================================
# top.v 的组合判定，单独提出来方便单测
# ============================================================================

def _command_stop_target(kind: int, duty: int, position_counts: int,
                         motion_mode: int, base_rpm: int) -> bool:
    """`command_stop_target`（top.v:126-132）逐条复刻。"""
    if kind == int(Kind.OPEN_LOOP):
        return duty == 0
    if kind == int(Kind.POSITION):
        return position_counts == 0
    if kind == int(Kind.HEARTBEAT):
        return True
    return motion_mode == 0 or base_rpm == 0


def _command_target_valid(kind: int, duty: int, position_counts: int,
                          position_max_rpm: int, motion_mode: int, base_rpm: int,
                          params: FirmwareParams) -> bool:
    """`command_target_valid`（top.v:133-147）逐条复刻。

    注意它对 KIND **没有 else 兜底**：STOP/ARM/DISARM 落到最后一条
    `(motion_mode <= 6) && (base_rpm <= 200)`，而这三条命令会把两者都清零，
    所以恒为真。这一条漏掉会让 ARM 直接锁死底盘。
    """
    if kind == int(Kind.OPEN_LOOP):
        return (motion_mode in (int(MotionMode.FORWARD), int(MotionMode.BACKWARD))
                and duty <= params.max_open_loop_duty)
    if kind == int(Kind.POSITION):
        return (motion_mode in (int(MotionMode.FORWARD), int(MotionMode.BACKWARD),
                                int(MotionMode.ROTATE_LEFT), int(MotionMode.ROTATE_RIGHT))
                and position_counts > params.pos_stop_tol_counts
                and params.position_rpm_min <= position_max_rpm <= params.position_rpm_max)
    if kind == int(Kind.HEARTBEAT):
        return True
    return motion_mode <= int(MotionMode.ROTATE_RIGHT) and base_rpm <= params.max_move_rpm


def _position_abort(cmd: dict, params: FirmwareParams) -> bool:
    """`position_abort`（top.v:121-125）。

    ⚠ **POSITION 自己不在中止列表里**：busy 期间再发一条 POSITION
      既不会重启也不会中止，什么都不会发生。
    """
    return cmd['kind'] in (int(Kind.STOP), int(Kind.MOVE),
                           int(Kind.DISARM), int(Kind.OPEN_LOOP))
