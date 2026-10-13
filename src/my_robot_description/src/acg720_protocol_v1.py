#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ACG720 底盘串口协议 V1 —— 按 FPGA 源码逐字节实现。

⚠️ 本文件取代 `acg720_protocol.py`。旧那份是**凭空发明**的：帧外层少了
   VER/SEQ/LEN 三个字节、CRC 用的是 MODBUS 而不是 CCITT-FALSE、
   0x05 当成"四轮速度"、0x84 当成"底盘遥测"。它跟真固件**说不上一句话**
   （第 3 个字节就会被判版本错）。迁移期间两份并存，迁移完成后删旧的。

================================================================================
权威来源
================================================================================
全部在交接包 `robot_car_handoff_20261006` 里（**包外，不在本仓库**）：

  · motor/wifi_car_position_loop_baseline_20261006/tools/chassis_control.py:17-59
        主机侧权威实现。本文件的编解码与它**逐字节等价**，并由
        test_protocol_v1.py 用它的黄金向量钉死。
  · .../output_file/chassis_command_rx.v:1-115, 240-295
        固件接收侧：帧状态机、CRC、LEN 上限、SEQ 规则、语义校验。
  · .../output_file/chassis_telemetry_imu_tx.v:37-51, 93-197, 240-302
        固件发送侧：5 种遥测、ACK、CRC、20 Hz。
  · .../output_file/chassis_safety_fsm.v:10-11, 56-62, 89-144
        安全状态机：ARM / 急停 / 300 ms 超时 / 3 s 上限 / 故障锁存。
  · .../output_file/chassis_motion.v:32-87
        MOVE 命令 → 四轮转速的展开方式。
  · navigation/src/robot_car_base/scripts/core.py:3-18
        导航侧独立参考实现（第三方交叉印证，与上面两者一致）。

================================================================================
帧格式（接收与发送**同构**）
================================================================================

    A5 5A | VER=0x01 | KIND | SEQ | LEN | PAYLOAD(LEN) | CRC16(低字节,高字节)

  · CRC-16/CCITT-FALSE：poly 0x1021、init 0xFFFF、**不反射**、**不异或输出**；
    覆盖 **VER..PAYLOAD**（**不含帧头 A5 5A**）；低字节先发。
    固件实现见 chassis_command_rx.v:54-70（逐位）,113,236-239。
    自检值：crc16(b"123456789") == 0x29B1。
  · LEN 只算 payload。
  · 115200 8N1。

================================================================================
三条会咬人的硬约束（驱动层必须按它们设计）
================================================================================

1. **任何一帧格式错 = 底盘被锁死在 FAULT，只能靠 S0/S4 复位序列解除。**
   CRC 错、VER 错、LEN 错、语义越界，任一都会置 `protocol_fault`
   （chassis_command_rx.v:240-247），由安全状态机**锁存**为 FAULT_PROTOCOL
   （chassis_safety_fsm.v:89-90,101-103），而 FAULT 状态是自锁的
   （`state <= STATE_FAULT`），没有"清除故障"命令。
   → 所以本模块的打包函数**在编码期就做语义校验**（见 MAX_* 与各 pack_*），
     宁愿在主机上抛异常，也不要把一帧越界数据发出去把车锁死。

2. **固件的 LEN 上限是 8，不是 48。** chassis_command_rx.v:208
   `if (rx_data > 8'd8) → protocol_fault`。
   主机侧参考实现允许 48（chassis_control.py:43），两者不是一回事。
   → 发命令一律用 `MAX_COMMAND_PAYLOAD`。

3. **SEQ 必须持续前进**：固件要求 `delta = SEQ - last_seq` 落在 1..127，
   否则整帧被**静默丢弃**（只加 duplicate 计数，不 ACK、不报错）
   —— chassis_command_rx.v:93-95,248-289。
   **唯一例外是 STOP(0x00)**：序号不新也执行（`sequence_new || stop_command`），
   这是故意的逃生口。
   注意：主机重启后 SEQ 归零并不会被自动接受 —— 必须先用 0x81 遥测里的
   `last_command_seq` 重新对齐（见 `SequenceTracker.resync_from_telemetry`）。
"""

from __future__ import annotations

import struct
from enum import IntEnum

# ============================================================================
# 帧常量
# ============================================================================

FRAME_HEAD = b"\xA5\x5A"
VERSION = 0x01

#: 固件接收侧真正接受的最大 payload（chassis_command_rx.v:208）。发命令用这个。
MAX_COMMAND_PAYLOAD = 8
#: 主机侧参考实现允许的上限（chassis_control.py:43）。仅用于接收/被动监听。
MAX_FRAME_PAYLOAD = 48

#: 最短帧长 = 帧头2 + VER1 + KIND1 + SEQ1 + LEN1 + CRC2（chassis_control.py:50）
MIN_FRAME_SIZE = 8


class Kind(IntEnum):
    """帧类型字节。命令 0x00-0x06，遥测/应答 0x80-0x84。"""

    STOP = 0x00
    ARM = 0x01
    MOVE = 0x02
    DISARM = 0x03
    OPEN_LOOP = 0x04
    POSITION = 0x05
    HEARTBEAT = 0x06

    ACK = 0x80
    TELEMETRY = 0x81            # 底盘：状态/故障/四轮目标/轮速/占空比/错误计数
    IMU = 0x82                  # JY901S 转发
    DIAGNOSTIC = 0x83           # 编码器诊断，含 build_id
    POSITION_TELEMETRY = 0x84   # 位置动作进度

    @property
    def is_command(self) -> bool:
        """固件只把 0x00-0x06 当命令解析（chassis_command_rx.v:46-52）。

        `command_kind` 输出只有 3 位（:250），所以 0x08 及以上永远过不了语义校验。
        """
        return 0x00 <= int(self) <= 0x06


class MotionMode(IntEnum):
    """0x02 MOVE / 0x05 POSITION 里的动作编码（chassis_motion.v:20-26）。"""

    STOP = 0
    FORWARD = 1
    BACKWARD = 2
    TURN_LEFT = 3               # 只有 0x02 MOVE 接受 3/4
    TURN_RIGHT = 4
    ROTATE_LEFT = 5
    ROTATE_RIGHT = 6


#: 0x05 POSITION 只接受这四种动作（chassis_command_rx.v:104-106）
POSITION_MODES = (MotionMode.FORWARD, MotionMode.BACKWARD,
                  MotionMode.ROTATE_LEFT, MotionMode.ROTATE_RIGHT)
#: 0x04 OPEN_LOOP 只接受直线（chassis_command_rx.v:101）
OPEN_LOOP_MODES = (MotionMode.FORWARD, MotionMode.BACKWARD)


class SafetyState(IntEnum):
    """安全状态机状态（chassis_safety_fsm.v:30-33）。"""

    DISARMED = 0
    ARMED_STOP = 1
    RUNNING = 2
    FAULT = 3


class FaultReason(IntEnum):
    """故障码（chassis_safety_fsm.v:47-51）。"""

    NONE = 0
    ESTOP = 1
    MOTOR = 2
    PROTOCOL = 3
    TIMEOUT = 4
    RUN_LIMIT = 5


class PositionState(IntEnum):
    """0x84 遥测 payload[1] 低 3 位（chassis_position_controller.v）。"""

    IDLE = 0
    RUN = 1
    SETTLE = 2
    DONE = 3
    ABORT = 4


class PositionResult(IntEnum):
    """0x84 遥测 payload[0] 的 bit5:2。"""

    NONE = 0
    OK = 1
    ABORTED = 2
    STALL = 3
    TIMEOUT = 4
    REJECTED = 5


# ============================================================================
# 取值范围（**必须在编码期拦住**，否则真机会锁进 FAULT）
# 依据：chassis_command_rx.v:97-112（semantic_valid）+ top.v:126-147（二次校验）
# ============================================================================

MAX_MOVE_RPM = 200              # 0x02 base_rpm 上限
MAX_OPEN_LOOP_DUTY = 1400       # 0x04 duty_cycles 上限
POSITION_RPM_MIN = 8            # 0x05 max_rpm 下限
POSITION_RPM_MAX = 40           # 0x05 max_rpm 上限
POSITION_MIN_COUNTS = 32        # 0x05 target_counts 必须 **严格大于** 32

#: 命令帧和它要求的 payload 长度（chassis_command_rx.v:110-112 等）
COMMAND_LENGTHS = {
    Kind.STOP: 0,
    Kind.ARM: 0,
    Kind.MOVE: 4,
    Kind.DISARM: 0,
    Kind.OPEN_LOOP: 4,
    Kind.POSITION: 8,
    Kind.HEARTBEAT: 0,
}

#: 遥测帧的 payload 长度（chassis_telemetry_imu_tx.v:244,251,263,280,289）
TELEMETRY_LENGTHS = {
    Kind.ACK: 2,
    Kind.TELEMETRY: 38,
    Kind.IMU: 36,
    Kind.DIAGNOSTIC: 42,
    Kind.POSITION_TELEMETRY: 16,
}


class ProtocolError(ValueError):
    """帧或载荷不合法。"""


# ============================================================================
# CRC-16/CCITT-FALSE
# ============================================================================

def _build_crc_table() -> tuple[int, ...]:
    table = []
    for byte in range(256):
        crc = byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        table.append(crc)
    return tuple(table)


_CRC_TABLE = _build_crc_table()


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE：poly 0x1021、init 0xFFFF、不反射、不异或输出。

    与固件 `crc16_byte()`（chassis_command_rx.v:54-70）逐位等价，也与主机侧
    `binascii.crc_hqx(body, 0xFFFF)` 等价 —— 后者是交接包用的写法，
    test_protocol_v1.py 会拿它做独立对照。
    """
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ byte) & 0xFF]
    return crc


# ============================================================================
# 帧编解码
# ============================================================================

def encode_frame(kind: int, sequence: int, payload: bytes = b"") -> bytes:
    """组一帧。等价于 chassis_control.py:42-46。

    `payload` 长度上限用 **MAX_FRAME_PAYLOAD**（48），因为本函数也用于生成
    测试向量；**发命令请走 `encode_command()`**，它按固件的 8 字节上限拦。
    """
    if len(payload) > MAX_FRAME_PAYLOAD:
        raise ProtocolError(f"payload {len(payload)} 字节，超过上限 {MAX_FRAME_PAYLOAD}")
    if not 0 <= sequence <= 0xFF:
        raise ProtocolError(f"SEQ {sequence} 不在 0..255")
    body = bytes((VERSION, int(kind) & 0xFF, sequence & 0xFF, len(payload))) + payload
    return FRAME_HEAD + body + struct.pack("<H", crc16(body))


def encode_command(kind: int, sequence: int, payload: bytes = b"") -> bytes:
    """组一条**命令**帧，并强制固件的真实限制。

    比 `encode_frame()` 多两道拦截：
      · payload 长度 ≤ MAX_COMMAND_PAYLOAD（8）—— 固件见 chassis_command_rx.v:208
      · kind 必须是 0x00-0x06 且 payload 长度与该命令的规定一致
        —— 固件见 chassis_command_rx.v:97-112
    这两条任意一条不满足，真机会把帧判为协议故障并**锁死底盘**，所以宁可
    在主机上抛异常。
    """
    k = Kind(int(kind))          # 非法的 kind 直接在这里抛 ValueError
    if not k.is_command:
        raise ProtocolError(f"0x{int(k):02X} 不是命令（固件只认 0x00-0x06）")
    if len(payload) > MAX_COMMAND_PAYLOAD:
        raise ProtocolError(
            f"payload {len(payload)} 字节 > 固件上限 {MAX_COMMAND_PAYLOAD}"
            f"（chassis_command_rx.v:208）")
    want = COMMAND_LENGTHS[k]
    if len(payload) != want:
        raise ProtocolError(f"{k.name} 要求 payload {want} 字节，给了 {len(payload)}")
    return encode_frame(k, sequence, payload)


def decode_frame(raw: bytes) -> tuple[int, int, bytes]:
    """拆一帧。等价于 chassis_control.py:49-59 与 core.py:decode()。

    返回 `(kind, sequence, payload)`；不合法抛 `ProtocolError`。
    """
    if len(raw) < MIN_FRAME_SIZE:
        raise ProtocolError(f"帧长 {len(raw)} < 最短 {MIN_FRAME_SIZE}")
    if raw[:2] != FRAME_HEAD:
        raise ProtocolError("帧头不是 A5 5A")
    if raw[2] != VERSION:
        raise ProtocolError(f"VER=0x{raw[2]:02X} ≠ 0x{VERSION:02X}")
    length = raw[5]
    if length > MAX_FRAME_PAYLOAD:
        raise ProtocolError(f"LEN={length} 超过上限 {MAX_FRAME_PAYLOAD}")
    if len(raw) != MIN_FRAME_SIZE + length:
        raise ProtocolError(f"帧长 {len(raw)} ≠ LEN+8 = {MIN_FRAME_SIZE + length}")
    expected = crc16(raw[2:-2])
    received = struct.unpack("<H", raw[-2:])[0]
    if expected != received:
        raise ProtocolError(f"CRC 错：算出 0x{expected:04X}，帧里是 0x{received:04X}")
    return raw[3], raw[4], raw[6:-2]


class FrameParser:
    """滚动帧解析器：喂字节 → 出完整帧；失步自动重同步并计数。

    与交接包 `chassis_wireless_probe.py:24-55` 的 `extract_frames()` 同构
    （语义照抄，独立实现，另加分类错误计数）。

    重同步的关键在两点：帧头 A5 后跟的不是 5A、而是又一个 A5 时**不退回去**
    （允许 `A5 A5 5A`），以及 CRC 错时只丢**一个字节**再从流里找下一个帧头。
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.discarded_bytes = 0     # 帧头之前的垃圾字节
        self.header_errors = 0       # VER 错或 LEN 越界
        self.length_errors = 0       # 帧长与 LEN 不符
        self.crc_errors = 0

    def reset(self) -> None:
        """串口重连时必须清 —— 否则半个残帧会污染重连后的第一帧。"""
        self._buf.clear()

    def feed(self, data: bytes) -> list[tuple[int, int, bytes]]:
        """喂一段字节，返回这一段里解析出的所有完整帧。"""
        self._buf.extend(data)
        out: list[tuple[int, int, bytes]] = []
        while True:
            start = self._buf.find(FRAME_HEAD)
            if start < 0:
                # 尾巴上可能挂着半个帧头，留着；其余全丢
                if self._buf[-1:] == FRAME_HEAD[:1]:
                    self.discarded_bytes += len(self._buf) - 1
                    del self._buf[:-1]
                else:
                    self.discarded_bytes += len(self._buf)
                    self._buf.clear()
                break
            if start:
                self.discarded_bytes += start
                del self._buf[:start]
            if len(self._buf) < 6:
                break                                # 还不够读到 LEN
            length = self._buf[5]
            if self._buf[2] != VERSION or length > MAX_FRAME_PAYLOAD:
                self.header_errors += 1
                del self._buf[0]                     # 只丢一个字节，重新找帧头
                continue
            size = MIN_FRAME_SIZE + length
            if len(self._buf) < size:
                break                                # 等后续字节
            raw = bytes(self._buf[:size])
            expected = crc16(raw[2:-2])
            received = struct.unpack_from("<H", raw, size - 2)[0]
            if expected != received:
                self.crc_errors += 1
                del self._buf[0]
                continue
            del self._buf[:size]
            out.append((raw[3], raw[4], raw[6:-2]))
        return out


# ============================================================================
# SEQ 规则
# ============================================================================

def seq_is_new(sequence: int, last_sequence: int) -> bool:
    """固件的 SEQ 新鲜度判据：`delta = SEQ - last` 落在 1..127。

    依据 chassis_command_rx.v:93-95：
        seq_delta    = seq_hold - last_seq
        sequence_new = (seq_delta != 0) && !seq_delta[7]
    注意 delta==0（重复）与 delta>=128（落后 ≥128）**都会被丢弃**。
    **STOP 是例外**，见 `stop_bypasses_sequence`。
    """
    delta = (sequence - last_sequence) & 0xFF
    return delta != 0 and delta < 128


def stop_bypasses_sequence(kind: int) -> bool:
    """STOP(0x00) 即使序号不新也会被固件执行（chassis_command_rx.v:248）。

    这是刻意的逃生口：序号乱了也能停车。
    """
    return int(kind) == Kind.STOP


class SequenceTracker:
    """主机侧 SEQ 管理。

    三个必须做对的地方：
      · 每条命令 SEQ+1（固件要求严格前进）
      · 首次通信固件是"来者不拒"（`!last_seq_valid`），之后才严格
      · **主机重启后不能从 0 硬来** —— 必须先用 0x81 遥测 payload[2] 的
        `last_command_seq` 对齐，否则新 SEQ 会被判成重复或旧帧而**静默丢弃**
        （chassis_command_rx.v:248,283-289），现象是"发了命令但车不动、
        也没有任何报错"。
    """

    def __init__(self, sequence: int = 0) -> None:
        self.sequence = sequence & 0xFF
        self.synced = False

    def take(self) -> int:
        """取当前 SEQ 并前进。发命令前调用，返回值就是这一帧要用的 SEQ。"""
        current = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        return current

    def resync_from_telemetry(self, last_command_seq: int) -> None:
        """按 0x81 遥测的 `last_command_seq` 对齐（等价于
        chassis_control.py:232 与 position_control.py:109）。

        +1 是因为固件记的是**它最后接受的那条**，我们要发下一条。
        """
        self.sequence = (int(last_command_seq) + 1) & 0xFF
        self.synced = True


# ============================================================================
# 命令载荷
# ============================================================================

def pack_move(mode: int, base_rpm: int, turn_inner_ratio_q8: int = 255) -> bytes:
    """0x02 MOVE：`mode u8 | base_rpm u16 LE | turn_inner_ratio_q8 u8`。

    ★ 这是当前固件**唯一**的连续运动命令，但它的表达能力有限
      （chassis_motion.v:32-87）：

        target_valid = (base_rpm <= 200) && (motion_mode <= 6)
        inner_rpm    = base_rpm * turn_inner_ratio_q8 / 256     ← 只能缩小
        outer_rpm    = base_rpm

        mode 1/2 FORWARD/BACKWARD : 四轮同速 ±base_rpm
        mode 3/4 TURN_LEFT/RIGHT  : 外侧 = base_rpm，内侧 = 上面那个 inner_rpm
        mode 5/6 ROTATE_LEFT/RIGHT: 左侧 ±base_rpm、右侧 ∓base_rpm

      **内侧轮速只能按 0..255 的比例缩小，不能反转、不能独立赋值。**
      所以 Nav2 那种"内侧轮要倒转"的 (v, ω) 用这条命令**表达不出来**，
      曲率也被量化成 256 档。这正是 INTERFACE.md 要新增 0x07 VELOCITY 的原因
      —— 而 0x07 目前**未分配、未实现**，发过去会锁死底盘。

    `turn_inner_ratio_q8` 缺省 255（≈直行），固件对这一项**不做范围检查**。
    """
    mode_i, rpm, ratio = int(mode), int(base_rpm), int(turn_inner_ratio_q8)
    if not 0 <= mode_i <= int(MotionMode.ROTATE_RIGHT):
        raise ProtocolError(f"mode {mode_i} 越界（固件要求 ≤6）")
    if not 0 <= rpm <= MAX_MOVE_RPM:
        raise ProtocolError(f"base_rpm {rpm} 越界（固件要求 ≤{MAX_MOVE_RPM}）")
    if not 0 <= ratio <= 0xFF:
        raise ProtocolError(f"turn_inner_ratio_q8 {ratio} 越界（0..255）")
    return struct.pack("<BHB", mode_i, rpm, ratio)


def parse_move(payload: bytes) -> dict:
    _need(payload, 4, Kind.MOVE)
    mode, rpm, ratio = struct.unpack("<BHB", payload)
    return {"motion_mode": mode, "base_rpm": rpm, "turn_inner_ratio_q8": ratio}


def pack_open_loop(mode: int, duty_cycles: int) -> bytes:
    """0x04 OPEN_LOOP：`mode u8 | duty u16 LE | reserved u8 = 0`。

    只允许直线（mode 1/2），duty 上限 1400（chassis_command_rx.v:100-103）。
    用途是台架标定，不是正常行驶。
    """
    mode_i, duty = int(mode), int(duty_cycles)
    if MotionMode(mode_i) not in OPEN_LOOP_MODES:
        raise ProtocolError(f"OPEN_LOOP 只接受 {[int(m) for m in OPEN_LOOP_MODES]}，"
                            f"给了 {mode_i}")
    if not 0 <= duty <= MAX_OPEN_LOOP_DUTY:
        raise ProtocolError(f"duty {duty} 越界（固件要求 ≤{MAX_OPEN_LOOP_DUTY}）")
    return struct.pack("<BHB", mode_i, duty, 0)


def parse_open_loop(payload: bytes) -> dict:
    _need(payload, 4, Kind.OPEN_LOOP)
    mode, duty, reserved = struct.unpack("<BHB", payload)
    return {"motion_mode": mode, "duty_cycles": duty, "reserved": reserved}


def pack_position(mode: int, max_rpm: int, target_counts: int) -> bytes:
    """0x05 POSITION：`mode u8 | max_rpm u8 | target_counts u32 LE | reserved u16 = 0`。

    这是**当前固件唯一能真正执行、且不受 3 s 上限约束**的运动命令：
    FPGA 自己闭环走完这段距离/转角，主机只需要每 100 ms 发一条新序号心跳
    （chassis_safety_fsm.v:57-58 里 `run_limit_reached` 带 `!position_active`）。

    约束（chassis_command_rx.v:104-109）：
      · mode ∈ {1 前进, 2 后退, 5 原地左旋, 6 原地右旋}
      · 8 ≤ max_rpm ≤ 40
      · target_counts **严格大于 32**
    动作开始时 FPGA 锁存四轮累计计数，之后只算相对增量，所以不用人工清零。
    """
    mode_i, rpm, counts = int(mode), int(max_rpm), int(target_counts)
    if MotionMode(mode_i) not in POSITION_MODES:
        raise ProtocolError(f"POSITION 只接受 {[int(m) for m in POSITION_MODES]}，"
                            f"给了 {mode_i}")
    if not POSITION_RPM_MIN <= rpm <= POSITION_RPM_MAX:
        raise ProtocolError(f"max_rpm {rpm} 越界"
                            f"（固件要求 {POSITION_RPM_MIN}..{POSITION_RPM_MAX}）")
    if not 0 <= counts <= 0xFFFFFFFF:
        raise ProtocolError(f"target_counts {counts} 超出 u32")
    if counts <= POSITION_MIN_COUNTS:
        raise ProtocolError(f"target_counts {counts} 必须 > {POSITION_MIN_COUNTS}"
                            f"（固件会判 REJECTED）")
    return struct.pack("<BBIH", mode_i, rpm, counts, 0)


def parse_position(payload: bytes) -> dict:
    _need(payload, 8, Kind.POSITION)
    mode, rpm, counts, reserved = struct.unpack("<BBIH", payload)
    return {"motion_mode": mode, "max_rpm": rpm,
            "target_counts": counts, "reserved": reserved}


def _need(payload: bytes, size: int, kind: Kind) -> None:
    if len(payload) != size:
        raise ProtocolError(f"{kind.name} 载荷应为 {size} 字节，实际 {len(payload)}")


# ============================================================================
# 遥测载荷
# ============================================================================
# 字段名与交接包 chassis_control.py 的解码函数保持一致，方便交叉核对。

#: 四轮顺序（top.v:4 注释）：A 右前、B 左前、C 左后、D 右后。
#: 即 A/D 在右侧、B/C 在左侧 —— 与 chassis_position_controller.v:126-133
#: 的原地左旋归一化（A、D 取正，B、C 取负）互相印证。
WHEEL_ORDER = ("A_right_front", "B_left_front", "C_left_rear", "D_right_rear")


def parse_ack(payload: bytes) -> dict:
    """0x80 ACK：`state u8 | fault u8`（chassis_telemetry_imu_tx.v:105-106）。

    帧的 SEQ 就是被 ACK 的那条命令的 SEQ（:244），所以可以用它确认"这条命令
    真的被解析了"。注意固件自己的契约：ACK **只证明帧被接受**，不证明动作已执行。

    这两个字节固件是零扩展的（`{6'd0, state}` / `{5'd0, fault}`），所以真帧里
    取值必然在范围内。这里**不做掩码**，与厂商参考实现保持逐字节等价
    —— 见 test_protocol_v1.py 里的活体交叉核对。
    """
    _need(payload, 2, Kind.ACK)
    return {"state": payload[0], "fault": payload[1]}


def parse_telemetry(payload: bytes) -> dict:
    """0x81 底盘遥测，38 字节，20 Hz。

    布局逐字节核对自 chassis_telemetry_imu_tx.v:109-131，与
    chassis_control.py:62-82 的解码完全一致。
    """
    _need(payload, 38, Kind.TELEMETRY)
    return {
        # state/fault/stby 在固件里都是零扩展的窄字段
        # （`{6'd0,state}` / `{5'd0,fault}` / `{7'd0,stby}`），所以真帧里必然在
        # 范围内。这里不掩码，与厂商参考实现逐字节等价。
        "state": payload[0],
        "fault": payload[1],
        # ★ 偏移 2 是"最后一条被接受的命令的 SEQ"，也是 0x81 帧自己的 SEQ
        #   （:250 packet_seq <= ack_seq，两处冗余）。它**不是**遥测计数器，
        #   所以不能拿它判断遥测丢帧。
        "last_command_seq": payload[2],
        "stby": payload[3],
        "target_rpm_abcd": list(struct.unpack_from("<4h", payload, 4)),
        "speed_rpm_abcd": list(struct.unpack_from("<4h", payload, 12)),
        "duty_cycles_abcd": list(struct.unpack_from("<4H", payload, 20)),
        "fault_abcd": list(payload[28:32]),
        "crc_errors": struct.unpack_from("<H", payload, 32)[0],
        "duplicates": struct.unpack_from("<H", payload, 34)[0],
        "frame_errors": struct.unpack_from("<H", payload, 36)[0],
    }


#: 0x82 里的 9 个 s16，顺序固定
_IMU_RAW_LAYOUT = ("accel", "gyro", "angle")


def parse_imu(payload: bytes) -> dict:
    """0x82 JY901S 转发，36 字节。布局见 chassis_telemetry_imu_tx.v:132-156
    与 chassis_control.py:85-119。换算系数也在那里（±16 g / ±2000 dps / ±180°）。
    """
    _need(payload, 36, Kind.IMU)
    flags = payload[0]
    raw = struct.unpack_from("<9h", payload, 4)
    accel, gyro, angle = raw[0:3], raw[3:6], raw[6:9]
    age_ms = struct.unpack_from("<H", payload, 24)[0]
    return {
        "flags": flags,
        "valid_accel": bool(flags & 0x01),
        "valid_gyro": bool(flags & 0x02),
        "valid_angle": bool(flags & 0x04),
        "fresh": bool(flags & 0x08),
        "stale": bool(flags & 0x10),
        "uart_error_seen": bool(flags & 0x20),
        "checksum_error_seen": bool(flags & 0x40),
        "frame_timeout_seen": bool(flags & 0x80),
        "last_frame_type": payload[1],
        "sample_counter": struct.unpack_from("<H", payload, 2)[0],
        "accel_raw_xyz": list(accel),
        "accel_g_xyz": [v * 16.0 / 32768.0 for v in accel],
        "gyro_raw_xyz": list(gyro),
        "gyro_dps_xyz": [v * 2000.0 / 32768.0 for v in gyro],
        "angle_raw_rpy": list(angle),
        "angle_deg_rpy": [v * 180.0 / 32768.0 for v in angle],
        "temperature_raw": struct.unpack_from("<h", payload, 22)[0],
        "temperature_c": struct.unpack_from("<h", payload, 22)[0] / 100.0,
        "age_ms": age_ms,
        "checksum_errors": struct.unpack_from("<H", payload, 26)[0],
        "frame_timeouts": struct.unpack_from("<H", payload, 28)[0],
        "uart_frame_errors": struct.unpack_from("<H", payload, 30)[0],
        "fpga_uptime_ms": struct.unpack_from("<I", payload, 32)[0],
    }


def parse_diagnostic(payload: bytes) -> dict:
    """0x83 编码器诊断，42 字节，20 Hz。

    ★ 这是**版本检测**的抓手：偏移 6 的 `build_id`
      （chassis_telemetry_imu_tx.v:40 当前为 0x20260A02）。
      交接包的 position_control.py:110-113 就是靠它拒绝跑错位流的：
      位流不是位置环版本就拒绝执行。FPGA 以后更新了，我们也靠它发现。

    `position_count_abcd` 是**绝对累计计数**，s32、会回绕 —— 里程计必须按
    模 2^32 取差量（core.py:delta32 的写法），不能当绝对值用。
    """
    _need(payload, 42, Kind.DIAGNOSTIC)
    build_id = struct.unpack_from("<I", payload, 6)[0]
    return {
        "diagnostic_counter": struct.unpack_from("<H", payload, 0)[0],
        "fpga_uptime_ms": struct.unpack_from("<I", payload, 2)[0],
        "build_id": build_id,
        "build_id_hex": f"{build_id:08X}",
        "position_count_abcd": list(struct.unpack_from("<4i", payload, 10)),
        "sample_delta_abcd": list(struct.unpack_from("<4h", payload, 26)),
        "invalid_transition_count_abcd": list(struct.unpack_from("<4H", payload, 34)),
    }


def parse_position_telemetry(payload: bytes) -> dict:
    """0x84 POSITION 遥测，16 字节，20 Hz。

    布局见 chassis_telemetry_imu_tx.v:178-197 与 chassis_control.py:137-154。
    """
    _need(payload, 16, Kind.POSITION_TELEMETRY)
    flags, meta = payload[0], payload[1]
    return {
        "busy": bool(flags & 0x01),
        "done": bool(flags & 0x02),
        "result_code": (flags >> 2) & 0x0F,
        "state": meta & 0x07,
        "motion_mode": (meta >> 3) & 0x07,
        "base_rpm": struct.unpack_from("<H", payload, 2)[0],
        "target_counts": struct.unpack_from("<I", payload, 4)[0],
        "progress_counts": struct.unpack_from("<i", payload, 8)[0],
        "remaining_counts": struct.unpack_from("<I", payload, 12)[0],
    }


#: 遥测类型 → (长度, 解码函数)
TELEMETRY_DECODERS = {
    Kind.ACK: parse_ack,
    Kind.TELEMETRY: parse_telemetry,
    Kind.IMU: parse_imu,
    Kind.DIAGNOSTIC: parse_diagnostic,
    Kind.POSITION_TELEMETRY: parse_position_telemetry,
}


def decode_telemetry_frame(kind: int, payload: bytes) -> dict:
    """按类型解码一帧遥测。未知类型抛 `ProtocolError`。"""
    try:
        decoder = TELEMETRY_DECODERS[Kind(int(kind))]
    except (ValueError, KeyError) as exc:
        raise ProtocolError(f"0x{int(kind):02X} 不是已知的遥测类型") from exc
    return decoder(payload)


# ============================================================================
# 里程计：累计计数 → 位移增量
# ============================================================================

def delta32(new: int, old: int) -> int:
    """s32 回绕安全的差量（与 core.py:delta32 同构）。

    FPGA 的 `position_count` 是 s32 累计值，会回绕；直接相减会在回绕点炸出
    一个 ±40 亿的假跳变。位置动作期间 FPGA 用"锁存起点 + 相对增量"规避，
    但里程计是连续量，必须自己做。
    """
    return ((new - old + 0x80000000) & 0xFFFFFFFF) - 0x80000000


def counts_to_metres(counts: float, counts_per_rev: float,
                     wheel_radius_m: float) -> float:
    """编码器计数 → 轮子滚过的距离（米）。"""
    return counts * 2.0 * 3.141592653589793 * wheel_radius_m / counts_per_rev
