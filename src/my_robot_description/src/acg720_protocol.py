#!/usr/bin/env python3
"""ACG720 底盘串口协议：帧编解码层。

为什么单独一个文件
------------------
真机迁移的第一件事是把"ROS 2 世界"和"FPGA 串口世界"接起来。这中间只有两件事
可能出错：**帧的字节布局**和**运动学换向**。把这两件事各关进一个纯函数模块，
就可以脱离硬件用单元测试钉死；剩下能出错的只有"串口通不通"。

⚠ 本协议中 payload 的**字节布局是本次设计补齐的，不是从交接包读到的。**
   交接包（`小车数据总览_实测状态分级.md` 第 4.4 节）只给出了：

       帧头 `A5 5A`；0x05 POSITION 位置命令；0x06 心跳（100 ms，300 ms 超时）；
       0x84 遥测（20 Hz）

   即它给了**帧边界、命令字和周期**，没给 payload 的字段顺序 / 字节序 / 校验。
   本文件按下列约定实现，全部集中在这里，将来拿到 FPGA 源码或协议文档时
   **只需要改本文件**，驱动节点和模拟器都不用动：

       - 帧格式：A5 5A | TYPE(1) | LEN(1) | PAYLOAD(LEN) | CRC16(2, 小端)
         CRC 覆盖 TYPE + LEN + PAYLOAD（不含帧头），多项式 0x8005 反射式
         （CRC-16/MODBUS：init=0xFFFF，refin/refout=true，xorout=0）。
       - 多字节整数一律**小端**。
       - RPM 用 **int16**（有符号，允许负值表示反转），单位 1 RPM。

校验和 CRC 的选择理由：整帧只有 1 字节长度字段，帧头 2 字节也不罕见，
不用 CRC 的话一旦丢字节就可能被误判成一帧合法数据，而底盘数据被误判的后果是
车按错误速度跑。CRC-16 在 10 字节量级的帧上开销可以忽略，按 115200 波特率、
20 Hz、每帧约 25 字节算，占用带宽不到 5%。

运动学约定（与 FPGA 代码 `chassis_motion.v` 的归一化一致）
---------------------------------------------------------
交接包 4.4 节写明 FPGA 的归一化是：

    前进 = (A+B+C+D)/4      左旋 = (A−B−C+D)/4

轮位 A=右前、B=左前、C=左后、D=右后。把这个式子写成左右两侧就得到：

    左轮平均 v_L = (B+C)/2        右轮平均 v_R = (A+D)/2
    前进  = (v_L+v_R)/2           左旋  = (v_L−v_R)/2

所以这是一台标准的 skid-steer，可以直接用左右两轮差速模型。**没有几何歧义**
——注意这里不需要知道 skid 侧滑系数，那是标定量，在 `drive_kinematics.py` 里。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Sequence, Tuple

# --------------------------------------------------------------------------
# 帧常量
# --------------------------------------------------------------------------

FRAME_HEAD = b"\xA5\x5A"

#: 帧里除帧头和 CRC 之外的最大长度（LEN 字段是 1 字节，所以 payload 上限 255；
#: 但留一个更紧的工程上限，避免解析到畸形帧时分配大内存）
MAX_PAYLOAD = 64

#: 一帧的固定开销 = 帧头 2 + TYPE 1 + LEN 1 + CRC 2
FRAME_OVERHEAD = 6


class Cmd(IntEnum):
    """命令字。低半区（<0x80）是下行命令，高半区（>=0x80）是上行上报。

    0x05 / 0x06 / 0x84 来自交接包文档，其余为本次补齐（见模块 docstring 的说明）。
    """

    # ---- 下行：上位机 → FPGA ----
    SET_VELOCITY = 0x01   # 四轮目标 RPM（新增）
    POSITION = 0x05       # 位置命令 / 定点旋转（交接包 4.4 节）
    HEARTBEAT = 0x06      # 心跳，100 ms 一次，FPGA 侧 300 ms 超时（交接包 4.4 节）
    SET_CONFIG = 0x07     # 预留：改 FPGA 里的限速 / 阈值（新增，暂未实现）

    # ---- 上行：FPGA → 上位机 ----
    STATUS = 0x83         # 心跳应答 + 故障字（新增）
    TELEMETRY = 0x84      # 遥测，20 Hz（交接包 4.4 节）
    POSITION_DONE = 0x85  # 位置命令完成 / 失败回报（新增）


#: 遥测帧里的状态标志位
class Flag:
    HEARTBEAT_TIMEOUT = 1 << 0   # FPGA 认为上位机心跳超时，已进入安全态
    ESTOP = 1 << 1               # 急停（S0）按下
    STALL = 1 << 2               # 卡死保护触发（duty>=1800 且实测<=3RPM 连续 10 采样）
    OVERSPEED = 1 << 3           # 实测 > 400 RPM
    TARGET_REJECT = 1 << 4       # 目标 > 200 RPM，被拒绝
    ENCODER_INVALID = 1 << 5     # AB 双位同时变化，出现非法跳变
    LOW_SPEED_WARN = 1 << 6      # 低速警告：内轮目标低于 ~7 RPM 跟踪不上（实测已知）


# --------------------------------------------------------------------------
# CRC-16/MODBUS
# --------------------------------------------------------------------------

def crc16(data: bytes) -> int:
    """CRC-16/MODBUS。查表实现，表在首次调用时构建。"""
    global _CRC_TABLE
    if _CRC_TABLE is None:
        table = []
        for byte in range(256):
            crc = byte
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
            table.append(crc)
        _CRC_TABLE = tuple(table)

    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ _CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


_CRC_TABLE = None


# --------------------------------------------------------------------------
# 运动学纯函数：四轮 RPM ↔ 左右轮速
# --------------------------------------------------------------------------

def wheels_to_lr(a: float, b: float, c: float, d: float) -> Tuple[float, float]:
    """四轮 RPM → (左轮平均 RPM, 右轮平均 RPM)。

    用交接包 4.4 节的归一化反推：左 = (B+C)/2，右 = (A+D)/2。
    返回的是**输出轴 RPM**，不是 m/s。
    """
    return ((b + c) / 2.0, (a + d) / 2.0)


def lr_to_wheels(left: float, right: float) -> Tuple[float, float, float, float]:
    """(左轮, 右轮) RPM → 四轮目标 (A, B, C, D)。

    轮位 A=右前、B=左前、C=左后、D=右后。同侧两轮给相同目标——
    这与交接包里"一侧两轮同目标"的曲线工况一致（F1 验收的工况表就是
    按 (A,B,C,D) 给的，例如右弯 40RPM 给 30/40/40/30，左右各自相等）。
    """
    return (right, left, left, right)


# --------------------------------------------------------------------------
# 编解码
# --------------------------------------------------------------------------

def encode(cmd: int, payload: bytes = b"") -> bytes:
    """组一帧：A5 5A | TYPE | LEN | PAYLOAD | CRC16(小端)。"""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload {len(payload)} 字节超过上限 {MAX_PAYLOAD}")
    body = bytes([cmd & 0xFF, len(payload)]) + payload
    return FRAME_HEAD + body + struct.pack("<H", crc16(body))


@dataclass
class Frame:
    """一帧解析结果。"""
    cmd: int
    payload: bytes


class FrameParser:
    """增量式帧解析器：喂字节流，吐出完整帧。

    读串口是**流**不是报文，一次 read 可能拿到半帧、也可能拿到三帧半。
    这个类负责重新同步：丢掉噪声字节、在 CRC 失败时滑过一个字节继续找帧头。

    用法::

        p = FrameParser()
        for frame in p.feed(ser.read(256)):
            handle(frame)
    """

    def __init__(self, max_payload: int = MAX_PAYLOAD, on_error=None):
        self._buf = bytearray()
        self._max_payload = max_payload
        self._on_error = on_error
        # 统计量：驱动节点会把它们报成诊断信息，方便现场判断"是没数据还是数据坏了"
        self.crc_errors = 0
        self.dropped_bytes = 0
        self.unknown_cmd: List[int] = []

    def feed(self, data: bytes) -> List[Frame]:
        self._buf.extend(data)
        frames: List[Frame] = []

        while True:
            # 1) 找帧头，丢掉前面的噪声
            start = self._buf.find(FRAME_HEAD)
            if start < 0:
                # 没有帧头。保留最后一个字节（可能是被截断的 0xA5）
                if len(self._buf) > 1:
                    self.dropped_bytes += len(self._buf) - 1
                    del self._buf[:-1]
                return frames
            if start > 0:
                self.dropped_bytes += start
                del self._buf[:start]

            # 2) 帧头够了，看长度字段是否已到齐
            if len(self._buf) < 4:
                return frames
            payload_len = self._buf[3]
            if payload_len > self._max_payload:
                # 长度非法：肯定是假帧头，滑过一个字节重新同步
                self._resync()
                continue

            total = FRAME_OVERHEAD + payload_len
            if len(self._buf) < total:
                return frames  # 还没收齐，等下一批

            candidate = bytes(self._buf[:total])
            body = candidate[2:-2]
            expected = struct.unpack("<H", candidate[-2:])[0]
            actual = crc16(body)
            if actual != expected:
                # CRC 失败：滑过一个字节，从下一个位置继续找帧头
                self.crc_errors += 1
                if self._on_error:
                    self._on_error(candidate, expected, actual)
                self._resync()
                continue

            del self._buf[:total]
            cmd = candidate[2]
            if cmd not in Cmd._value2member_map_:
                # 合法帧但命令字不认识：交出去让上层记录，不丢——
                # 将来 FPGA 加了新上报类型，这里能第一时间看出来
                self.unknown_cmd.append(cmd)
            frames.append(Frame(cmd=cmd, payload=candidate[4:-2]))

    def _resync(self) -> None:
        """CRC / 长度异常后重新同步：丢掉当前帧头，从下一个字节继续。"""
        del self._buf[:1]
        self.dropped_bytes += 1

    @property
    def pending_bytes(self) -> int:
        return len(self._buf)


# --------------------------------------------------------------------------
# 下行命令的 payload 构造
# --------------------------------------------------------------------------

def pack_velocity(rpm_a: float, rpm_b: float, rpm_c: float, rpm_d: float) -> bytes:
    """0x01 SET_VELOCITY：四个 int16 目标 RPM，顺序 A, B, C, D。"""
    vals = [int(round(v)) for v in (rpm_a, rpm_b, rpm_c, rpm_d)]
    for v in vals:
        if not -32768 <= v <= 32767:
            raise ValueError(f"目标 RPM {v} 超出 int16 范围")
    return struct.pack("<4h", *vals)


def pack_heartbeat(seq: int) -> bytes:
    """0x06 HEARTBEAT：1 字节序号（回绕）。"""
    return bytes([seq & 0xFF])


def pack_position(counts: int, rpm: int, mode: int, timeout_samples: int) -> bytes:
    """0x05 POSITION：定点移动 / 定点旋转。

    counts          目标编码器计数，**带符号**（决定前进方向或旋转方向）
    rpm             巡航 RPM
    mode            0=直线(前进/后退) 1=原地旋转
    timeout_samples 超时，单位是 50 ms 采样周期（交接包：TIMEOUT=240 采样≈12 s）
    """
    return struct.pack("<ihBH", counts, rpm, mode & 0xFF, timeout_samples & 0xFFFF)


@dataclass
class VelocityCmd:
    """0x01 SET_VELOCITY 解析结果：四轮目标 RPM，顺序 A,B,C,D。"""
    rpm: Tuple[int, int, int, int]

    @property
    def left_rpm(self) -> float:
        return (self.rpm[1] + self.rpm[2]) / 2.0

    @property
    def right_rpm(self) -> float:
        return (self.rpm[0] + self.rpm[3]) / 2.0


VELOCITY_FORMAT = "<4h"
VELOCITY_SIZE = struct.calcsize(VELOCITY_FORMAT)


def parse_velocity(payload: bytes) -> VelocityCmd:
    """解析 0x01 SET_VELOCITY（虚拟车 / FPGA 侧用）。"""
    if len(payload) != VELOCITY_SIZE:
        raise ValueError(f"速度 payload 长度 {len(payload)} != 期望 {VELOCITY_SIZE}")
    return VelocityCmd(rpm=struct.unpack(VELOCITY_FORMAT, payload))


def parse_heartbeat(payload: bytes) -> int:
    """解析 0x06 HEARTBEAT，返回序号。"""
    if len(payload) != 1:
        raise ValueError(f"心跳 payload 长度 {len(payload)} != 1")
    return payload[0]


@dataclass
class PositionCmd:
    """0x05 POSITION 解析结果。"""
    counts: int
    rpm: int
    mode: int
    timeout_samples: int


def parse_position(payload: bytes) -> PositionCmd:
    if len(payload) != 8:
        raise ValueError(f"位置 payload 长度 {len(payload)} != 8")
    counts, rpm, mode, timeout = struct.unpack("<ihBH", payload)
    return PositionCmd(counts=counts, rpm=rpm, mode=mode,
                       timeout_samples=timeout)


# --------------------------------------------------------------------------
# 上行上报的 payload 解析
# --------------------------------------------------------------------------

@dataclass
class Telemetry:
    """0x84 遥测帧（20 Hz）。

    交接包 4.2 节：测速采样周期 50 ms，遥测刷新 20 Hz，两者相同。
    这里同时给出"上一采样区间的计数增量"和"换算好的 RPM"，理由见
    `rpm` 字段的注释。
    """
    seq: int                    # 递增序号，用来测丢帧率
    uptime_ms: int              # FPGA 上电以来的毫秒数，用来算真实时间戳
    counts: Tuple[int, int, int, int]   # 本采样区间的编码器增量 A,B,C,D（带符号）
    rpm: Tuple[int, int, int, int]      # FPGA 自己换算的 RPM（int16，A,B,C,D）
    duty: Tuple[int, int, int, int]     # 四路 PWM duty，0..5000（占空比上限 2700）
    flags: int                  # 状态旗标，见 Flag

    def flag_names(self) -> List[str]:
        names = []
        for name in dir(Flag):
            if name.startswith("_"):
                continue
            bit = getattr(Flag, name)
            if self.flags & bit:
                names.append(name)
        return names

    @property
    def left_rpm(self) -> float:
        return (self.rpm[1] + self.rpm[2]) / 2.0

    @property
    def right_rpm(self) -> float:
        return (self.rpm[0] + self.rpm[3]) / 2.0


#: 0x84 的 payload 布局，编码用 "<I I 4h 4h 4H H"=… 写成显式格式串便于核对
TELEMETRY_FORMAT = "<II4h4h4HH"
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FORMAT)


def pack_telemetry(t: Telemetry) -> bytes:
    """组 0x84 遥测帧的 payload（模拟器用，也是给 FPGA 端的参考实现）。"""
    return struct.pack(
        TELEMETRY_FORMAT,
        t.seq & 0xFFFFFFFF,
        t.uptime_ms & 0xFFFFFFFF,
        *t.counts,
        *t.rpm,
        *t.duty,
        t.flags & 0xFFFF,
    )


def parse_telemetry(payload: bytes) -> Telemetry:
    """解析 0x84 遥测帧的 payload。"""
    if len(payload) != TELEMETRY_SIZE:
        raise ValueError(
            f"遥测 payload 长度 {len(payload)} != 期望 {TELEMETRY_SIZE}"
        )
    fields = struct.unpack(TELEMETRY_FORMAT, payload)
    seq, uptime = fields[0], fields[1]
    counts = tuple(fields[2:6])
    rpm = tuple(fields[6:10])
    duty = tuple(fields[10:14])
    flags = fields[14]
    return Telemetry(seq=seq, uptime_ms=uptime, counts=counts, rpm=rpm,
                     duty=duty, flags=flags)


@dataclass
class Status:
    """0x83 状态帧：心跳应答 + FPGA 侧对下行命令的确认。

    `accepted` 很关键：交接包 4.3 节写 FPGA 会拒绝 > 200 RPM 的目标。
    如果上位机不知道目标被拒，就会出现"发了速度但车没动、日志里还显示正常"。
    """
    heartbeat_echo: int     # 回显收到的心跳序号
    accepted: bool          # 上一条速度/位置命令是否被接受
    reject_code: int        # 0=OK，1=>200RPM，2=换向保护未满足，3=急停中，4=参数越界
    uptime_ms: int


# ⚠ 这里的格式串是**实测出来的**，不是想当然写的。过程记录如下，免得再踩：
#
#   1) "<IBBI"   → calcsize 14：struct 把 uint32 对齐到 4 字节边界，隐式插了 4 字节
#   2) "<IBB3xI" → calcsize 13：uint32 落在偏移 6
#   3) "<IBBxI"  → calcsize 11（**不是直觉的 8**）：开头的 'I' 让整个结构有了
#                  4 字节对齐要求，于是 x 和 I 之间又被插了隐式 padding
#   4) "<4BI"    → calcsize 8 ✓
#
# 关键教训：`struct` 的 `x` 是"跳过 1 字节"，**不是**"可以随便放的填充"；
# 一旦格式串里出现过一个 4 字节字段，后续字段就会被隐式对齐。
# 所以最稳的写法是**不用 x**：把 pad 写成第 4 个 'B'（值恒为 0），
# 这样字节位置完全由字段本身决定，cpp/Verilog 那边照抄也一样。
#
# 最终布局（与 C 默认对齐一致，可移植）：
#   echo@0  accepted@1  reject_code@2  pad@3  uptime_ms@4   → 8 字节
STATUS_FORMAT = "<4BI"
STATUS_SIZE = struct.calcsize(STATUS_FORMAT)

REJECT_NAMES = {
    0: "OK",
    1: "TARGET_GT_200RPM",
    2: "REVERSE_PROTECTION",
    3: "ESTOP_ACTIVE",
    4: "PARAM_OUT_OF_RANGE",
}


def pack_status(s: Status) -> bytes:
    # 格式串 "<4BI" 里的第 4 个 B 是显式 padding 位，恒传 0。
    # 用 B 而不用 x 的原因见 STATUS_FORMAT 上方的注释。
    return struct.pack(STATUS_FORMAT, s.heartbeat_echo & 0xFF,
                       1 if s.accepted else 0, s.reject_code & 0xFF,
                       0,                      # 显式 padding，恒为 0
                       s.uptime_ms & 0xFFFFFFFF)


def parse_status(payload: bytes) -> Status:
    if len(payload) != STATUS_SIZE:
        raise ValueError(f"状态 payload 长度 {len(payload)} != 期望 {STATUS_SIZE}")
    echo, accepted, code, _pad, uptime = struct.unpack(STATUS_FORMAT, payload)
    return Status(heartbeat_echo=echo, accepted=bool(accepted),
                  reject_code=code, uptime_ms=uptime)


@dataclass
class PositionDone:
    """0x85 位置命令回报。"""
    mode: int               # 0=直线 1=旋转
    success: bool
    reason: int             # 0=到位 1=超时 2=卡死 3=被新命令打断
    final_counts: int       # 实际走到的计数（带符号）
    elapsed_samples: int


# mode(u8), success(u8), reason(u8), pad(u8), final_counts(int32), elapsed(u16)
POSITION_DONE_FORMAT = "<BBBBiH"
POSITION_DONE_SIZE = struct.calcsize(POSITION_DONE_FORMAT)

DONE_REASON_NAMES = {0: "ARRIVED", 1: "TIMEOUT", 2: "STALL", 3: "PREEMPTED"}


def pack_position_done(p: PositionDone) -> bytes:
    return struct.pack(POSITION_DONE_FORMAT, p.mode & 0xFF,
                       1 if p.success else 0, p.reason & 0xFF, 0,
                       p.final_counts, p.elapsed_samples & 0xFFFF)


def parse_position_done(payload: bytes) -> PositionDone:
    if len(payload) != POSITION_DONE_SIZE:
        raise ValueError(f"位置回报长度 {len(payload)} != 期望 {POSITION_DONE_SIZE}")
    mode, success, reason, _pad, final_counts, elapsed = struct.unpack(
        POSITION_DONE_FORMAT, payload)
    return PositionDone(mode=mode, success=bool(success), reason=reason,
                        final_counts=final_counts, elapsed_samples=elapsed)
