#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""固件行为模型的单测：每一条都在钉住"真固件会怎么反应"。

为什么这个文件重要
==================
`test_protocol_v1.py` 钉的是"字节对不对"。本文件钉的是**行为**：

    · 哪一种帧会被判非法 → 底盘永久锁死
    · ARM / 300 ms 命令超时 / 3.0 s 连续运动上限 / 故障自锁
    · ACK 单槽撞车 → tx_overflow → 也是永久锁死
    · 位置环怎么算完成、什么时候判堵转/超时
    · 哪些 (v, ω) MOVE 根本表达不出来

这些每一条在真车上试错的代价都是"车锁死 + 拆电复位"，
所以必须在这里用毫秒级的纯函数测试钉死。

★ 本文件**刻意**断言那些"看着像 bug"的行为
  （HEARTBEAT 会刹停、重发 MOVE 不清零 run_age、内侧轮不能反转）。
  它们不是 bug，是固件的真实设计。谁把它们"修好"，谁就得先改这份测试 ——
  而改测试的时候就会看到理由。

跑法：
    pytest src/my_robot_description/test/test_firmware_model.py -q
"""

import math
import os
import struct
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, '..', 'src')
sys.path.insert(0, _SRC)

from acg720_firmware import (  # noqa: E402
    EV_ACCEPTED, EV_CRC_ERROR, EV_DUPLICATE, EV_FRAME_TIMEOUT, EV_LENGTH_ERROR,
    EV_SEMANTIC_ERROR, EV_UART_FRAME_ERROR, EV_VERSION_ERROR,
    Acg720Firmware, FirmwareParams, PositionLoop, RxFsm, SafetyFsm,
    WHEEL_A_RIGHT_FRONT, WHEEL_B_LEFT_FRONT, WHEEL_C_LEFT_REAR,
    WHEEL_D_RIGHT_REAR, _command_stop_target, _command_target_valid,
    _counts_to_rpm, _position_abort, chassis_motion,
)
from acg720_protocol_v1 import (  # noqa: E402
    FrameParser, FaultReason, Kind, MotionMode, PositionResult, PositionState,
    SafetyState, SequenceTracker, encode_command, encode_frame, pack_move,
    pack_open_loop, pack_position, parse_ack, parse_diagnostic, parse_imu,
    parse_position_telemetry, parse_telemetry,
)

# ============================================================================
# 测试台：固件模型 + 最小主机
# ============================================================================


class Rig:
    """把固件模型接上一个"最小主机"：自己管 SEQ、自己解析回来的帧。

    时间只由 `advance()` 显式推进，不用墙上时钟，所以测试完全确定。
    """

    def __init__(self, **param_overrides):
        self.fw = Acg720Firmware(FirmwareParams(**param_overrides))
        self.seq = SequenceTracker()
        self.parser = FrameParser()
        self.frames = []            # [(kind, seq, payload)]
        self.bad_frames = []        # 解析失败的原始字节
        self.t = 0.0
        self.last_seq = None

    # ---- 下行 ----

    def feed(self, blob):
        """发任意字节，返回接收事件名列表。"""
        return self.fw.feed(blob)

    def send(self, kind, payload=b'', sequence=None):
        """组一条合法命令帧发出去，返回接收事件名列表。"""
        s = self.seq.take() if sequence is None else sequence
        self.last_seq = s
        return self.fw.feed(encode_command(kind, s, payload))

    def send_raw(self, blob):
        """发任意字节（用来构造固件会判非法的帧）。"""
        return self.fw.feed(blob)

    # ---- 时间 ----

    def advance(self, dt=0.02):
        data = self.fw.tick(dt) + self.fw.drain_tx()
        self.t += dt
        if data:
            self.frames.extend(self.parser.feed(data))
        return data

    def run(self, duration, dt=0.02):
        end = self.t + duration
        while self.t < end - 1e-12:
            self.advance(dt)

    def run_periodic(self, duration, period, make, dt=0.02):
        """按固定周期发命令同时推进。

        `make(rig)` 返回 `(kind, payload)` 或 None（None = 这一拍不发）。
        ⚠ 用"下一次到点的绝对时刻"来排程，不要写 `t % period` ——
          浮点取模会随机漏发，然后测试就变成在测 300 ms 超时了。
        """
        end = self.t + duration
        next_at = self.t
        while self.t < end - 1e-12:
            if self.t >= next_at - 1e-12:
                item = make(self)
                if item is not None:
                    self.send(item[0], item[1])
                next_at += period
            self.advance(dt)

    # ---- 上行 ----

    def frames_of(self, kind):
        return [f for f in self.frames if f[0] == int(kind)]

    def last_of(self, kind):
        got = self.frames_of(kind)
        return got[-1] if got else None

    def ack_count(self):
        return len(self.frames_of(Kind.ACK))

    def arm(self):
        self.send(Kind.ARM)
        self.run(0.06)


def payload_of(rig, kind):
    frame = rig.last_of(kind)
    assert frame is not None, f"没有收到 0x{int(kind):02X} 帧"
    return frame[2]


# ============================================================================
# 一、帧层：接受与拒绝
# ============================================================================

#: 七条命令各一条合法帧（用 host 侧打包函数生成，与厂商实现已交叉核对过）
def _all_command_frames():
    return [
        (Kind.STOP, b''),
        (Kind.ARM, b''),
        (Kind.DISARM, b''),
        (Kind.HEARTBEAT, b''),
        (Kind.MOVE, pack_move(int(MotionMode.FORWARD), 100, 128)),
        (Kind.OPEN_LOOP, pack_open_loop(int(MotionMode.FORWARD), 1000)),
        (Kind.POSITION, pack_position(int(MotionMode.FORWARD), 25, 3232)),
    ]


@pytest.mark.parametrize('kind,payload', _all_command_frames())
def test_all_seven_commands_are_accepted(kind, payload):
    rig = Rig()
    events = rig.send_raw(encode_command(kind, 7, payload))
    assert EV_ACCEPTED in events
    rig.advance()
    ack = rig.last_of(Kind.ACK)
    assert ack is not None, '被接受的命令必须回 ACK'
    assert ack[1] == 7, 'ACK 帧的 SEQ 必须是被 ACK 那条命令的 SEQ'
    info = parse_ack(ack[2])
    assert info['state'] == int(SafetyState.DISARMED)
    assert info['fault'] == int(FaultReason.NONE)


def test_ack_reflects_accepted_sequence_not_a_counter():
    rig = Rig()
    rig.send_raw(encode_command(Kind.HEARTBEAT, 0x40))
    rig.advance()
    rig.send_raw(encode_command(Kind.HEARTBEAT, 0x41))
    rig.advance()
    assert [f[1] for f in rig.frames_of(Kind.ACK)] == [0x40, 0x41]


def test_header_resync_a5_a5_a5_5a():
    """`A5 A5 A5 5A` 必须能自恢复（chassis_command_rx.v:181-183）。

    写成"不是 5A 就回 HEADER_0"会让这种流永久失步。
    """
    rig = Rig()
    frame = encode_command(Kind.STOP, 3)
    events = rig.send_raw(b'\xA5' + frame)
    assert EV_ACCEPTED in events
    assert rig.fw.rx.crc_errors == 0


def test_leading_garbage_bytes_are_skipped():
    rig = Rig()
    events = rig.send_raw(b'\x00\xFF\x12' + encode_command(Kind.STOP, 4))
    assert EV_ACCEPTED in events
    assert rig.fw.rx.crc_errors == 0


def test_frame_split_across_reads_is_reassembled():
    rig = Rig()
    frame = encode_command(Kind.STOP, 5)
    for chunk in (frame[:3], frame[3:6], frame[6:]):
        rig.send_raw(chunk)
    assert rig.fw.rx.crc_errors == 0
    assert rig.fw.rx.last_seq_valid and rig.fw.rx.last_seq == 5


def test_version_mismatch_latches_fault():
    """VERSION 错 → protocol_fault（:191）。**不是**丢一帧了事。"""
    rig = Rig()
    frame = bytearray(encode_command(Kind.STOP, 0))
    frame[2] = 0x02
    events = rig.send_raw(bytes(frame))
    assert EV_VERSION_ERROR in events
    rig.advance()
    assert rig.fw.state == SafetyState.FAULT
    assert rig.fw.fault == FaultReason.PROTOCOL
    assert rig.fw.latched


def test_len_over_eight_latches_fault():
    """固件 LEN 上限是 8（:208），不是 INTERFACE.md 里写的 48。"""
    rig = Rig()
    body = bytes((1, int(Kind.HEARTBEAT), 0, 9)) + bytes(range(9))
    rig.send_raw(b'\xA5\x5A' + body + struct.pack('<H', 0))
    rig.advance()
    assert rig.fw.state == SafetyState.FAULT
    assert rig.fw.fault == FaultReason.PROTOCOL


def test_len_zero_command_with_nonzero_kind_is_semantic_error():
    """ARM 的 LEN 必须是 0；给它塞 payload 会被判语义错。"""
    rig = Rig()
    rig.send_raw(encode_frame(Kind.ARM, 0, b'\x00\x00\x00\x00'))
    rig.advance()
    assert rig.fw.state == SafetyState.FAULT
    assert rig.fw.fault == FaultReason.PROTOCOL


def test_crc_error_latches_fault_and_counts():
    rig = Rig()
    frame = bytearray(encode_command(Kind.STOP, 0))
    frame[-1] ^= 0xFF
    events = rig.send_raw(bytes(frame))
    assert EV_CRC_ERROR in events
    rig.advance()
    assert rig.fw.rx.crc_errors == 1
    assert rig.fw.fault == FaultReason.PROTOCOL


def test_crc_error_frame_still_allows_later_frames_to_be_parsed():
    rig = Rig()
    bad = bytearray(encode_command(Kind.STOP, 0))
    bad[-1] ^= 0xFF
    rig.send_raw(bytes(bad))
    rig.advance()
    # 已经锁死，但接收机本身仍在工作：再来一帧能解析出来（只是状态救不回来）
    rig.send_raw(encode_command(Kind.HEARTBEAT, 1))
    rig.advance()
    assert rig.fw.rx.crc_errors == 1
    assert rig.fw.state == SafetyState.FAULT


def test_intra_frame_timeout_does_not_set_fault():
    """帧内字节间隔 >100 ms → 丢半个帧 + frame_error_count，**不置 fault**（:156-160）。

    这是四类非法里唯一不锁死底盘的一类，很容易被写成"也报警"。
    """
    rig = Rig()
    frame = encode_command(Kind.STOP, 0)
    rig.send_raw(frame[:5])
    rig.run(0.15)                      # 远超 100 ms
    assert rig.fw.rx.frame_errors == 1
    assert rig.fw.state != SafetyState.FAULT
    # 残帧已经丢掉：把剩下的字节补上也不会被当成一帧
    rig.send_raw(frame[5:])
    rig.advance()
    assert rig.fw.rx.last_seq_valid is False


def test_frame_timeout_event_is_reported():
    rig = Rig()
    frame = encode_command(Kind.STOP, 0)
    rig.fw.feed(frame[:5])
    events = []
    for _ in range(8):
        rig.fw.tick(0.02)
        events += rig.fw.events
    assert EV_FRAME_TIMEOUT in events


def test_uart_frame_error_latches_fault():
    """UART 停止位错/break → protocol_fault（:165-170）。"""
    rig = Rig()
    rig.fw.rx.inject_uart_frame_error()
    rig.advance()
    assert rig.fw.rx.frame_errors == 1
    assert rig.fw.fault == FaultReason.PROTOCOL


# ============================================================================
# 二、语义校验：越界一律锁死
# ============================================================================

def _raw(kind, payload, sequence=0):
    """绕过 encode_command 的拦截，直接组一帧越界数据。"""
    return encode_frame(kind, sequence, payload)


@pytest.mark.parametrize('name,blob', [
    ('MOVE rpm=201', _raw(Kind.MOVE, struct.pack('<BHB', 1, 201, 255))),
    ('MOVE mode=7', _raw(Kind.MOVE, struct.pack('<BHB', 7, 100, 255))),
    ('OPEN_LOOP duty=1401', _raw(Kind.OPEN_LOOP, struct.pack('<BHB', 1, 1401, 0))),
    ('OPEN_LOOP mode=3', _raw(Kind.OPEN_LOOP, struct.pack('<BHB', 3, 500, 0))),
    ('OPEN_LOOP ratio!=0', _raw(Kind.OPEN_LOOP, struct.pack('<BHB', 1, 500, 9))),
    ('POSITION rpm=7', _raw(Kind.POSITION, struct.pack('<BBIH', 1, 7, 3232, 0))),
    ('POSITION rpm=41', _raw(Kind.POSITION, struct.pack('<BBIH', 1, 41, 3232, 0))),
    ('POSITION counts=32', _raw(Kind.POSITION, struct.pack('<BBIH', 1, 25, 32, 0))),
    ('POSITION reserved!=0', _raw(Kind.POSITION, struct.pack('<BBIH', 1, 25, 3232, 1))),
    ('POSITION mode=3', _raw(Kind.POSITION, struct.pack('<BBIH', 3, 25, 3232, 0))),
    ('未知 KIND 0x07', _raw(0x07, b'')),
    ('未知 KIND 0x10', _raw(0x10, b'')),
])
def test_semantically_invalid_frames_latch_fault(name, blob):
    rig = Rig()
    events = rig.send_raw(blob)
    assert EV_SEMANTIC_ERROR in events, f'{name} 应当被判语义错，实际 {events}'
    rig.advance()
    assert rig.fw.state == SafetyState.FAULT, f'{name} 应当锁死底盘'
    assert rig.fw.fault == FaultReason.PROTOCOL


@pytest.mark.parametrize('payload', [
    struct.pack('<BHB', 1, 100, 255),      # MOVE 边界内
    struct.pack('<BHB', 2, 200, 0),        # rpm 正好 200
    struct.pack('<BHB', 6, 1, 128),
])
def test_boundary_values_are_accepted(payload):
    rig = Rig()
    assert EV_ACCEPTED in rig.send_raw(_raw(Kind.MOVE, payload))


def test_position_rpm_check_uses_payload_byte_one_not_the_16bit_value():
    """POSITION 的转速检查用 payload[1] 单字节（:107）。

    第 2 个字节在 POSITION 里属于计数低位，若误用 16 位拼值就会把
    "rpm=25、计数低位=200" 这种**合法**帧判成 rpm>40 而锁死底盘。
    """
    payload = struct.pack('<BBIH', 1, 25, 3232, 0)
    assert payload[1] == 25 and payload[2] == 0xA0      # 3232 = 0x0CA0
    rig = Rig()
    assert EV_ACCEPTED in rig.send_raw(_raw(Kind.POSITION, payload))


# ============================================================================
# 三、SEQ 规则
# ============================================================================

def test_duplicate_sequence_is_dropped_without_ack():
    """重复 SEQ 只加计数，**不回 ACK、不置 fault**（:287-289）。"""
    rig = Rig()
    rig.send(Kind.HEARTBEAT, sequence=5)
    rig.advance()
    before = rig.ack_count()
    events = rig.send(Kind.HEARTBEAT, sequence=5)
    assert EV_DUPLICATE in events
    rig.advance()
    assert rig.ack_count() == before
    assert rig.fw.rx.duplicates == 1
    assert rig.fw.state != SafetyState.FAULT


def test_stop_bypasses_sequence_check():
    """STOP 是逃生口：序号不新也执行（:248）。"""
    rig = Rig()
    rig.arm()
    rig.send(Kind.HEARTBEAT, sequence=100)
    rig.advance()
    events = rig.send(Kind.STOP, sequence=100)      # 同样的 SEQ
    assert EV_ACCEPTED in events


def test_sequence_delta_128_is_treated_as_stale():
    rig = Rig()
    rig.send(Kind.HEARTBEAT, sequence=0)
    rig.advance()
    events = rig.send(Kind.HEARTBEAT, sequence=128)
    assert EV_DUPLICATE in events


def test_sequence_wrap_255_to_0_is_new():
    rig = Rig()
    rig.send(Kind.HEARTBEAT, sequence=255)
    rig.advance()
    events = rig.send(Kind.HEARTBEAT, sequence=0)
    assert EV_ACCEPTED in events


def test_first_frame_is_always_accepted():
    """`!last_seq_valid`：上电后第一帧来者不拒（:94）。"""
    rig = Rig()
    assert EV_ACCEPTED in rig.send(Kind.HEARTBEAT, sequence=99)


# ============================================================================
# 四、安全状态机
# ============================================================================

def test_disarmed_silently_ignores_everything_but_arm():
    """DISARMED 下非 ARM 命令**静默忽略**（:120-126），不报错不置 fault。

    现象是"发了命令但车不动、也没有任何告警" —— 现场最容易误判成串口坏了。
    """
    for kind, payload in ((Kind.MOVE, pack_move(1, 100)),
                          (Kind.HEARTBEAT, b''),
                          (Kind.STOP, b''),
                          (Kind.OPEN_LOOP, pack_open_loop(1, 500))):
        rig = Rig()
        events = rig.send(kind, payload)
        assert EV_ACCEPTED in events            # 帧本身是合法的
        rig.advance()
        assert rig.fw.state == SafetyState.DISARMED
        assert rig.fw.fault == FaultReason.NONE


def test_arm_moves_to_armed_stop():
    rig = Rig()
    rig.arm()
    assert rig.fw.state == SafetyState.ARMED_STOP
    assert rig.fw.safety.armed


def test_second_arm_is_harmless():
    """ARM 的 payload 恒为空，所以 `command_stop_target` 恒为真,
    ARMED_STOP 下再发 ARM **不会**触发 FAULT。

    ⚠ 早期笔记里写过"ARMED_STOP 下带非零目标再 ARM → FAULT"，
      那条路径其实**不可达**：ARM 要求 LEN=0，而 chasssis_command_rx.v:273-279
      的 else 分支会把 motion_mode/base_rpm 清零，于是带目标的 ARM 根本构造不出来。
    """
    rig = Rig()
    rig.arm()
    rig.send(Kind.ARM)
    rig.advance()
    assert rig.fw.state == SafetyState.ARMED_STOP
    assert rig.fw.fault == FaultReason.NONE


def test_move_arms_running_from_armed_stop():
    rig = Rig()
    rig.arm()
    rig.send(Kind.MOVE, pack_move(int(MotionMode.FORWARD), 100))
    rig.advance()
    assert rig.fw.state == SafetyState.RUNNING
    assert rig.fw.target_rpm() == [100, 100, 100, 100]


def test_zero_target_move_only_moves_to_armed_stop():
    rig = Rig()
    rig.arm()
    rig.send(Kind.MOVE, pack_move(int(MotionMode.FORWARD), 0))
    rig.advance()
    assert rig.fw.state == SafetyState.ARMED_STOP


def test_disarm_returns_to_disarmed():
    rig = Rig()
    rig.arm()
    rig.send(Kind.DISARM)
    rig.advance()
    assert rig.fw.state == SafetyState.DISARMED


def test_command_timeout_latches_fault_after_300ms():
    rig = Rig()
    rig.arm()
    rig.run(0.2)
    assert rig.fw.state == SafetyState.ARMED_STOP
    rig.run(0.2)                     # 累计 >300 ms 没有有效命令
    assert rig.fw.state == SafetyState.FAULT
    assert rig.fw.fault == FaultReason.TIMEOUT


def test_no_command_timeout_while_disarmed():
    rig = Rig()
    rig.run(2.0)
    assert rig.fw.state == SafetyState.DISARMED
    assert rig.fw.fault == FaultReason.NONE


def test_heartbeat_keeps_armed_alive_indefinitely():
    rig = Rig()
    rig.arm()
    rig.run_periodic(2.0, 0.1, lambda r: (Kind.HEARTBEAT, b''))
    assert rig.fw.state == SafetyState.ARMED_STOP
    assert rig.fw.fault == FaultReason.NONE


def test_fault_is_self_latching_and_no_command_clears_it():
    """FAULT 自锁（:88）。唯一出路是 `reset_n`（按住 S4）。"""
    rig = Rig()
    rig.arm()
    rig.run_periodic(3.6, 0.1, lambda r: (Kind.MOVE, pack_move(1, 100)), dt=0.02)
    assert rig.fw.state == SafetyState.FAULT

    for kind, payload in ((Kind.STOP, b''), (Kind.ARM, b''), (Kind.DISARM, b''),
                          (Kind.HEARTBEAT, b''), (Kind.MOVE, pack_move(1, 50))):
        rig.send_raw(encode_command(kind, rig.seq.take(), payload))
        rig.advance()
        assert rig.fw.state == SafetyState.FAULT, f'{kind} 不该能清除 FAULT'
    assert rig.fw.fault == FaultReason.RUN_LIMIT
    assert rig.fw.target_rpm() == [0, 0, 0, 0]


def test_timeout_and_run_limit_are_distinguishable():
    """单发一条 MOVE 后不再发命令：触发的是 300 ms **超时**，不是 3 s 上限。

    两条都锁死底盘，但"是超时还是跑太久"决定了该改主机还是该改 FPGA，
    所以必须分得清。
    """
    rig = Rig()
    rig.arm()
    rig.send(Kind.MOVE, pack_move(1, 100))
    rig.run(1.0, dt=0.02)
    assert rig.fw.state == SafetyState.FAULT
    assert rig.fw.fault == FaultReason.TIMEOUT


def test_reset_clears_fault():
    rig = Rig()
    rig.arm()
    rig.run_periodic(3.6, 0.1, lambda r: (Kind.MOVE, pack_move(1, 100)), dt=0.02)
    assert rig.fw.latched
    rig.fw.reset()
    assert rig.fw.state == SafetyState.DISARMED
    assert rig.fw.fault == FaultReason.NONE


def test_estop_latches_fault_and_only_reset_recovers():
    """急停拉低 → `estop_seen` 锁存 → FAULT（:69-72, 81-85）。

    ★ 松开急停**回不来**：`estop_seen` 只在 `reset_n` 为低（按住 S4）且
      `estop_n` 已经为高时才清零。所以恢复动作是"**先松 S0，再按 S4**"，
      光松 S0 什么都不会发生。
    """
    rig = Rig()
    rig.arm()
    rig.fw.safety.set_estop(False)
    rig.advance()
    assert rig.fw.state == SafetyState.FAULT
    assert rig.fw.fault == FaultReason.ESTOP

    rig.fw.safety.set_estop(True)          # 松开 S0
    rig.advance()
    assert rig.fw.state == SafetyState.FAULT, '光松 S0 不该恢复'

    rig.fw.safety.set_reset(False)         # 再按住 S4
    rig.advance()
    assert rig.fw.state == SafetyState.DISARMED
    assert rig.fw.safety.estop_seen is False

    rig.fw.safety.set_reset(True)
    rig.advance()
    assert rig.fw.state == SafetyState.DISARMED


def test_motor_fault_branch():
    rig = Rig()
    rig.fw.safety.inject_motor_fault(True)
    rig.advance()
    assert rig.fw.state == SafetyState.FAULT
    assert rig.fw.fault == FaultReason.MOTOR


# ============================================================================
# 五、3.0 s 连续运动上限 —— 本项目的**决定性**限制
# ============================================================================

def test_run_limit_latches_even_with_continuous_move_stream():
    """★ 核心断言：**10 Hz 连续重发 MOVE 也躲不过 3.0 s 上限。**

    `run_age` 只在离开 RUNNING 时清零（:72-77），重发命令不清零；
    而 FAULT_RUN_LIMIT 一旦置起就自锁。

    这就是"主机侧把 (v, ω) 反解成 MOVE 就万事大吉"这条路走不通的原因，
    也是 `docs/fpga_change_request.md` 里 P0 那条变更的全部理由。
    """
    rig = Rig()
    rig.arm()
    rig.run_periodic(3.6, 0.1, lambda r: (Kind.MOVE, pack_move(1, 100)), dt=0.02)
    assert rig.fw.state == SafetyState.FAULT
    assert rig.fw.fault == FaultReason.RUN_LIMIT


def test_run_age_is_not_reset_by_resending_move():
    rig = Rig()
    rig.arm()
    rig.send(Kind.MOVE, pack_move(1, 100))
    rig.run_periodic(2.0, 0.1, lambda r: (Kind.MOVE, pack_move(1, 100)), dt=0.02)
    age_before = rig.fw.safety.run_age_s
    rig.run_periodic(0.5, 0.1, lambda r: (Kind.MOVE, pack_move(1, 100)), dt=0.02)
    assert rig.fw.safety.run_age_s > age_before, \
        'run_age 必须继续增长；如果被重发清零，那 3 s 上限就成了摆设'


def test_stop_then_move_restarts_run_age_which_is_why_cycling_is_forbidden():
    """STOP 确实会让 `run_age` 清零 —— 这正是"周期 STOP 绕过"能 work 的原理，
    也是它被明令禁止的原因。这里把机制钉住，免得有人无意中重新发明它。
    """
    rig = Rig()
    rig.arm()
    rig.run_periodic(2.0, 0.1, lambda r: (Kind.MOVE, pack_move(1, 100)), dt=0.02)
    assert rig.fw.safety.run_age_s > 1.5
    rig.send(Kind.STOP)
    rig.advance()
    assert rig.fw.state == SafetyState.ARMED_STOP
    rig.advance()                 # run_age 的更新在下一拍才看到状态已离开 RUNNING
    assert rig.fw.safety.run_age_s == 0.0


def test_run_age_accumulates_during_position_but_the_trigger_is_masked():
    """★ 位置环期间 `run_age` **照常累积**，被屏蔽的只是"触发"（`:57` 的
    `!position_active`）。

    所以一次 >3 s 的位置动作结束时，`run_age` 早就超过 3 s 了。它没有变成 FAULT，
    是因为位置环同时发出 `internal_stop_pulse`，而 `internal_stop` 在 FSM 的
    优先级链里排在 `run_limit_reached` **前面**（:92-96）。
    这个顺序要是反了，**每一次超过 3 秒的位置动作都会把底盘锁死** ——
    而位置动作本来就常常超过 3 秒。
    """
    rig = Rig()
    rig.arm()
    rig.send(Kind.POSITION, pack_position(int(MotionMode.FORWARD), 25, 3232))
    rig.run_periodic(3.5, 0.1, lambda r: (Kind.HEARTBEAT, b''), dt=0.02)
    assert rig.fw.state != SafetyState.FAULT
    assert rig.fw.position.busy, '位置动作还没走完'
    assert rig.fw.safety.run_age_s > 3.0, 'run_age 应当一直在涨'
    assert not rig.fw.safety.run_limit_reached, '但触发条件被 position_active 屏蔽'


def test_position_finishing_after_three_seconds_lands_in_armed_stop_not_fault():
    rig = Rig()
    rig.arm()
    rig.send(Kind.POSITION, pack_position(int(MotionMode.FORWARD), 25, 3232))
    rig.run_periodic(9.0, 0.1, lambda r: (Kind.HEARTBEAT, b''), dt=0.02)
    assert rig.fw.position.result_code == PositionResult.OK
    assert rig.fw.state == SafetyState.ARMED_STOP
    assert rig.fw.safety.run_age_s == 0.0


def test_faulted_target_is_zero():
    rig = Rig()
    rig.arm()
    rig.run_periodic(3.6, 0.1, lambda r: (Kind.MOVE, pack_move(1, 100)), dt=0.02)
    assert rig.fw.latched
    assert rig.fw.target_rpm() == [0, 0, 0, 0]


# ============================================================================
# 六、HEARTBEAT 会刹停运动 —— top.v 的坑
# ============================================================================

def test_heartbeat_while_running_zeroes_the_manual_target():
    """★ `top.v:238-281` 的 `active_*` 只认"MOVE 且目标非零"。

    其它任何被接受的命令（含 HEARTBEAT）都走 `else` 分支把目标清零。
    所以"100 ms 心跳喂狗 + 持续发 MOVE"的设计在真机上会把车**刹停**，
    而状态机还停在 RUNNING —— 现象是"车走走停停，日志一切正常"。
    """
    rig = Rig()
    rig.arm()
    rig.run_periodic(0.6, 0.1, lambda r: (Kind.MOVE, pack_move(int(MotionMode.FORWARD), 100)),
                     dt=0.02)
    assert rig.fw.state == SafetyState.RUNNING
    assert rig.fw.target_rpm() == [100, 100, 100, 100]

    rig.send(Kind.HEARTBEAT)
    rig.advance()
    assert rig.fw.state == SafetyState.RUNNING, '状态没变 …'
    assert rig.fw.target_rpm() == [0, 0, 0, 0], '… 但目标被清零了'


def test_resending_move_keeps_motion_and_feeds_the_watchdog():
    """正确的喂狗方式是**重发 MOVE**（它同样刷新 command_age）。"""
    rig = Rig()
    rig.arm()
    rig.run_periodic(2.0, 0.1, lambda r: (Kind.MOVE, pack_move(1, 100)), dt=0.02)
    assert rig.fw.state == SafetyState.RUNNING
    assert rig.fw.target_rpm() == [100, 100, 100, 100]
    assert rig.fw.safety.command_age_s < 0.2


# ============================================================================
# 七、发送侧：ACK 单槽与 tx_overflow
# ============================================================================

def test_tx_overflow_latches_fault_when_ack_waits_behind_telemetry():
    """★ `tx_overflow`（:236-238）→ `top.v:231` OR 进 protocol_fault → **永久锁死**。

    触发条件不是"命令发得快"，而是"**第二条被接受的命令落在一条在途周期遥测的
    窗口里**"：此时 ACK 槽被占住最长 ~14 ms（20 Hz 的四个包共 164 字节），
    第二条命令一来就撞车。

    所以主机限速的判据是"命令周期要明显大于一整批遥测的发送时间"，
    而不是"ACK 帧有多长"。
    """
    rig = Rig()
    rig.arm()
    # 推到"刚好有一批周期遥测在发送"的时刻。批量的起点是 50 ms 的整数倍，
    # 但 dt=20 ms 时累加器会在 60 ms 才到点，所以按状态找而不是按时间猜。
    for _ in range(20):
        rig.advance(0.005)
        if rig.fw.tx.packet_active:
            break
    assert rig.fw.tx.packet_active, '此时应当有一条遥测正在发送'

    rig.send(Kind.HEARTBEAT)                 # ACK 想发，但排在在途包后面
    rig.advance(0.002)
    assert rig.fw.tx.pending['ack'], 'ACK 槽此时应当被占住'
    rig.send(Kind.HEARTBEAT)                 # 槽还被占 → 撞车
    rig.advance(0.002)
    assert rig.fw._tx_overflow_count == 1
    assert rig.fw.state == SafetyState.FAULT
    assert rig.fw.fault == FaultReason.PROTOCOL


def test_back_to_back_command_frames_alone_do_not_overflow():
    """两帧紧挨着的命令**不会**溢出 —— ACK 只有 10 字节，槽位很快就腾空。

    把限制想得过紧和想得过松一样有害：这条是为了防止有人据此把
    命令周期无谓地放到几百毫秒（那会反过来撞上 300 ms 超时）。
    """
    rig = Rig()
    blob = encode_command(Kind.HEARTBEAT, 0) + encode_command(Kind.HEARTBEAT, 1)
    events = rig.feed(blob)
    assert events.count(EV_ACCEPTED) == 2
    rig.run(0.02)
    assert rig.fw._tx_overflow_count == 0
    assert rig.ack_count() == 2


def test_normal_ten_hz_command_stream_never_overflows():
    """10 Hz 命令 + 20 Hz 遥测下，ACK 槽永远来得及腾空。"""
    rig = Rig()
    rig.arm()
    rig.run_periodic(3.0, 0.1, lambda r: (Kind.HEARTBEAT, b''), dt=0.02)
    assert rig.fw._tx_overflow_count == 0
    assert rig.fw.state == SafetyState.ARMED_STOP
    assert rig.ack_count() >= 25


def test_telemetry_is_emitted_at_twenty_hertz():
    rig = Rig()
    rig.run(1.0)
    for kind, lower in ((Kind.TELEMETRY, 18), (Kind.IMU, 18),
                        (Kind.DIAGNOSTIC, 18), (Kind.POSITION_TELEMETRY, 18)):
        got = len(rig.frames_of(kind))
        assert got >= lower, f'0x{int(kind):02X} 只发了 {got} 帧'


def test_telemetry_frames_round_trip_through_the_decoders():
    """0x81/0x83/0x84/0x82 用**真协议解析器**解得回来，字段语义也对。"""
    rig = Rig()
    rig.arm()
    rig.run_periodic(0.5, 0.1, lambda r: (Kind.MOVE, pack_move(int(MotionMode.FORWARD), 50)),
                     dt=0.02)
    assert rig.fw.state == SafetyState.RUNNING

    tele = parse_telemetry(payload_of(rig, Kind.TELEMETRY))
    assert tele['state'] == int(SafetyState.RUNNING)
    assert tele['target_rpm_abcd'] == [50, 50, 50, 50]
    assert tele['last_command_seq'] == rig.fw.rx.last_seq

    diag = parse_diagnostic(payload_of(rig, Kind.DIAGNOSTIC))
    assert diag['build_id_hex'] == '20260A02'
    assert diag['build_id'] == rig.fw.build_id
    # 四个轮子朝同一方向转，累计计数应当同号
    assert all(c > 0 for c in diag['position_count_abcd'])

    pos = parse_position_telemetry(payload_of(rig, Kind.POSITION_TELEMETRY))
    assert set(pos) >= {'busy', 'done', 'result_code', 'state', 'motion_mode'}

    imu = parse_imu(payload_of(rig, Kind.IMU))
    assert imu['stale'] is True, '台架上没有 IMU，必须报 stale —— 绝不伪造有效数据'
    assert imu['valid_angle'] is False


def test_telemetry_seq_semantics():
    """0x81 的 SEQ = 最后被接受的命令 SEQ；0x83/0x84 的 SEQ 是同一个诊断计数器。"""
    rig = Rig()
    rig.arm()
    rig.send(Kind.HEARTBEAT, sequence=0x33)
    rig.run(0.3)
    assert rig.last_of(Kind.TELEMETRY)[1] == 0x33
    diag_seqs = {f[1] for f in rig.frames_of(Kind.DIAGNOSTIC)}
    pos_seqs = {f[1] for f in rig.frames_of(Kind.POSITION_TELEMETRY)}
    assert diag_seqs, '至少要有诊断帧'
    assert pos_seqs <= diag_seqs, '0x83 与 0x84 共用一个计数器，SEQ 集合应当一致'


# ============================================================================
# 八、位置环
# ============================================================================

def test_position_reaches_target_and_reports_ok():
    rig = Rig()
    rig.arm()
    rig.send(Kind.POSITION, pack_position(int(MotionMode.FORWARD), 25, 3232))
    rig.run_periodic(9.0, 0.1, lambda r: (Kind.HEARTBEAT, b''), dt=0.02)
    assert rig.fw.position.state == PositionState.DONE
    assert rig.fw.position.result_code == PositionResult.OK
    progress = rig.fw.position.progress_counts
    assert abs(progress - 3232) <= 40, f'走到 {progress}，目标是 3232'


def test_position_progress_is_monotonic_and_remaining_shrinks():
    rig = Rig()
    rig.arm()
    rig.send(Kind.POSITION, pack_position(int(MotionMode.FORWARD), 25, 1000))
    seen = []
    end = rig.t + 5.0
    while rig.t < end and rig.fw.position.busy:
        rig.send(Kind.HEARTBEAT)
        rig.advance()
        seen.append((rig.fw.position.progress_counts, rig.fw.position.remaining_counts))
    assert len(seen) > 20
    assert all(seen[i][0] <= seen[i + 1][0] for i in range(len(seen) - 1))
    assert seen[0][1] > seen[-1][1]


def test_position_spins_up_and_down_through_the_slow_zones():
    """转速剖面：远处 max_rpm → 慢区 15 → 蠕行 8 → 停止容差内 0。"""
    rig = Rig()
    rig.arm()
    rig.send(Kind.POSITION, pack_position(int(MotionMode.FORWARD), 25, 3232))
    rpms = []
    end = rig.t + 9.0
    while rig.t < end and rig.fw.position.busy:
        rig.send(Kind.HEARTBEAT)
        rig.advance()
        rpms.append(rig.fw.position.base_rpm)
    assert max(rpms) == 25
    assert 15 in rpms, '应当经过慢区（15 RPM）'
    assert 8 in rpms, '应当经过蠕行区（8 RPM）'
    assert rpms[-1] == 0


def test_position_target_counts_semantics_is_per_wheel_average():
    """`target_counts` 是**四个轮子增量的平均**（`>>2`，:78），不是求和。

    这条搞错的话，标定出来的"每米多少计数"会差 4 倍。
    """
    rig = Rig()
    rig.arm()
    rig.send(Kind.POSITION, pack_position(int(MotionMode.FORWARD), 25, 600))
    rig.run_periodic(4.0, 0.1, lambda r: (Kind.HEARTBEAT, b''), dt=0.02)
    counts = rig.fw.plant.position_counts
    progress = rig.fw.position.progress_counts
    assert abs(progress - sum(counts) / 4.0) <= 2.0, \
        f'progress={progress} 应当≈四轮平均={sum(counts)/4.0}'


def test_position_timeout_reports_timeout():
    rig = Rig(pos_timeout_ticks=5)
    rig.arm()
    rig.send(Kind.POSITION, pack_position(int(MotionMode.FORWARD), 40, 100000))
    rig.run_periodic(1.0, 0.1, lambda r: (Kind.HEARTBEAT, b''), dt=0.02)
    assert rig.fw.position.result_code == PositionResult.TIMEOUT
    assert rig.fw.position.busy is False


def test_position_stall_is_detected():
    """电机几乎不转 → 10 个采样 tick 后判堵转。"""
    rig = Rig(motor_time_constant_s=1e6)
    rig.arm()
    rig.send(Kind.POSITION, pack_position(int(MotionMode.FORWARD), 25, 3232))
    rig.run_periodic(2.0, 0.1, lambda r: (Kind.HEARTBEAT, b''), dt=0.02)
    assert rig.fw.position.result_code == PositionResult.STALL


def test_position_busy_ignores_a_second_position_command():
    """★ busy 期间再发 POSITION **什么都不会发生**：既不重启也不中止。

    因为 `position_start` 只看 kind，而 `position_abort`（top.v:121-125）
    只认 STOP/MOVE/DISARM/OPEN_LOOP —— POSITION 自己不在列表里。
    """
    rig = Rig()
    rig.arm()
    rig.send(Kind.POSITION, pack_position(int(MotionMode.FORWARD), 25, 3000))
    rig.run(1.0)
    before = rig.fw.position.target_counts_latched
    rig.send(Kind.POSITION, pack_position(int(MotionMode.FORWARD), 25, 200))
    rig.advance()
    assert rig.fw.position.target_counts_latched == before
    assert rig.fw.position.result_code == PositionResult.NONE
    assert rig.fw.position.busy


def test_move_during_position_aborts_it():
    rig = Rig()
    rig.arm()
    rig.send(Kind.POSITION, pack_position(int(MotionMode.FORWARD), 25, 3000))
    rig.run_periodic(1.0, 0.1, lambda r: (Kind.HEARTBEAT, b''), dt=0.02)
    assert rig.fw.position.busy
    rig.send(Kind.MOVE, pack_move(int(MotionMode.FORWARD), 100))
    rig.run_periodic(0.2, 0.1, lambda r: (Kind.MOVE, pack_move(int(MotionMode.FORWARD), 100)),
                     dt=0.02)
    assert rig.fw.position.result_code == PositionResult.ABORTED
    assert rig.fw.position.busy is False
    assert rig.fw.state == SafetyState.RUNNING


def test_position_done_returns_state_to_armed_stop():
    """走完 → `internal_stop_pulse` → RUNNING→ARMED_STOP（:92-96）。"""
    rig = Rig()
    rig.arm()
    rig.send(Kind.POSITION, pack_position(int(MotionMode.FORWARD), 25, 400))
    rig.run_periodic(4.0, 0.1, lambda r: (Kind.HEARTBEAT, b''), dt=0.02)
    assert rig.fw.position.result_code == PositionResult.OK
    assert rig.fw.state == SafetyState.ARMED_STOP


def test_position_valid_start_gate():
    loop = PositionLoop(FirmwareParams())
    assert loop.valid_start(int(MotionMode.FORWARD), 100, 8)
    assert loop.valid_start(int(MotionMode.ROTATE_LEFT), 100, 40)
    assert not loop.valid_start(int(MotionMode.FORWARD), 32, 25)      # 计数必须 >32
    assert not loop.valid_start(int(MotionMode.FORWARD), 100, 7)      # rpm <8
    assert not loop.valid_start(int(MotionMode.FORWARD), 100, 41)     # rpm >40
    assert not loop.valid_start(int(MotionMode.TURN_LEFT), 100, 25)   # 3/4 不接受


def test_position_rejected_path_sets_rejected():
    """`valid_start` 失败时走 RESULT_REJECTED 分支（:197-206）。

    ⚠ 这条路径**从线上到不了**：`semantic_valid` 的判据与 `valid_start` 完全一致，
      能过前者的帧必然过后后者。这里直接调 API 把分支覆盖住。
    """
    loop = PositionLoop(FirmwareParams())
    loop.start(int(MotionMode.FORWARD), 32, 25, [0, 0, 0, 0])
    assert loop.result_code == PositionResult.REJECTED
    assert loop.done and not loop.busy


# ============================================================================
# 九、被控对象与编码器换算
# ============================================================================

def test_counts_to_rpm_matches_rtl_fixed_point():
    """`RPM = (计数×238313)>>18`，且 |计数| 先饱和到 32767（encoder_quadrature.v:53,105-129）。"""
    assert _counts_to_rpm(1320) == 1200          # 一圈/50 ms = 1200 RPM
    assert _counts_to_rpm(44) == 40              # 40 RPM 对应 44 计数
    assert _counts_to_rpm(-44) == -40
    assert _counts_to_rpm(0) == 0
    assert _counts_to_rpm(100000) == _counts_to_rpm(32767), '必须先饱和'
    # 与教科书公式 10/11 的差别在 1 RPM 以内
    assert abs(_counts_to_rpm(1320) - 1320 * 10 / 11) <= 1


def test_low_speed_floor_reproduces_handoff_defect():
    """目标低于 ≈7 RPM 时车以 7 RPM 滑行（交接包 F1 实测）。

    ⚠ 必须看**一段时间的平均**。50 ms 窗口里 7 RPM 只对应约 7.7 个编码器计数，
      整数截断会让单次上报在 5~9 RPM 之间跳 —— 这是真系统就有的量化抖动
      （低速下编码器分辨率不够），不是模型缺陷。
    """
    rig = Rig()
    rig.arm()
    move = lambda r: (Kind.MOVE, pack_move(int(MotionMode.FORWARD), 3))
    rig.run_periodic(1.0, 0.1, move, dt=0.02)
    samples = []
    for _ in range(20):
        rig.run_periodic(0.1, 0.1, move, dt=0.02)
        samples.append(rig.fw.plant.measured_rpm[WHEEL_A_RIGHT_FRONT])
    mean_rpm = sum(samples) / len(samples)
    assert abs(mean_rpm - 7) <= 1.0, f'平均 {mean_rpm:.2f} RPM 应当≈7'
    assert rig.fw.plant.floor_sliding_samples > 0
    assert set(samples) != {0}, '轮子真的在转'


def test_ground_truth_skid_scale_shrinks_rotation():
    """真值有效轮距 = 几何 × scale，"编码器以为转了 1 圈、车实际只转 0.73 圈"。

    ⚠ 必须在**稳态窗口**里比。拿"累计转角 / 瞬时角速度"会被起步瞬态污染
      （一阶响应前 0.12 s + 斜坡），本项目第一版就这么错过。
    """
    scale = 1.36
    rig = Rig(ground_truth_separation_scale=scale)
    rig.arm()
    move = lambda r: (Kind.MOVE, pack_move(int(MotionMode.ROTATE_LEFT), 20))
    rig.run_periodic(0.6, 0.1, move, dt=0.02)      # 起转并进入稳态
    y0, t0 = rig.fw.plant.yaw, rig.t
    rig.run_periodic(0.8, 0.1, move, dt=0.02)
    w_true = (rig.fw.plant.yaw - y0) / (rig.t - t0)

    rpm_diff = (rig.fw.plant.rpm_f[WHEEL_A_RIGHT_FRONT]
                - rig.fw.plant.rpm_f[WHEEL_B_LEFT_FRONT])
    w_naive = rpm_diff * 2 * math.pi * 0.0325 / 60.0 / 0.200
    assert abs(w_true / w_naive - 1.0 / scale) < 0.03, \
        f'比值 {w_true / w_naive:.4f} 应当≈{1.0 / scale:.4f}'


def test_rotate_left_yaw_is_positive():
    rig = Rig()
    rig.arm()
    rig.send(Kind.MOVE, pack_move(int(MotionMode.ROTATE_LEFT), 20))
    rig.run(1.0)
    assert rig.fw.plant.yaw > 0.0


def test_straight_line_moves_forward_positive_x():
    rig = Rig()
    rig.arm()
    rig.send(Kind.MOVE, pack_move(int(MotionMode.FORWARD), 30))
    rig.run(1.0)
    assert rig.fw.plant.x > 0.0
    assert abs(rig.fw.plant.y) < 1e-6


def test_motor_is_disabled_outside_running():
    rig = Rig()
    rig.arm()
    rig.run(0.5)
    assert rig.fw.target_rpm() == [0, 0, 0, 0]
    assert rig.fw.plant.rpm_f[0] == 0.0


# ============================================================================
# 十、纯组合逻辑：chassis_motion 与 top.v 的两个判定
# ============================================================================

def test_chassis_motion_shrinks_inner_wheel_only():
    """mode 3/4：内侧轮 = base×ratio>>8，**无符号**，只能缩小不能反转。"""
    outer, inner, inner2, outer2 = chassis_motion(int(MotionMode.TURN_LEFT), 100, 128)
    assert (outer, outer2) == (100, 100)
    assert inner == inner2 == (100 * 128) >> 8 == 50
    # ratio=0 时内侧轮停转（这就是"最小转弯半径 = 半轮距"的来源）
    assert chassis_motion(int(MotionMode.TURN_LEFT), 100, 0)[1] == 0
    # 无论 ratio 多大，内侧轮都不会变成负数
    assert all(v >= 0 for v in chassis_motion(int(MotionMode.TURN_RIGHT), 100, 255))


def test_chassis_motion_rotation_is_opposite_signs():
    a, b, c, d = chassis_motion(int(MotionMode.ROTATE_LEFT), 20, 0)
    assert (a, b, c, d) == (20, -20, -20, 20)
    a, b, c, d = chassis_motion(int(MotionMode.ROTATE_RIGHT), 20, 0)
    assert (a, b, c, d) == (-20, 20, 20, -20)


def test_chassis_motion_backward_has_no_arc_variant():
    """倒退只有直线（mode 2），**没有**倒退转弯 —— 这是 4B 的一处能力缺口。"""
    assert chassis_motion(int(MotionMode.BACKWARD), 50, 200) == (-50, -50, -50, -50)


def test_chassis_motion_target_valid_gate():
    assert chassis_motion(int(MotionMode.FORWARD), 200, 0) == (200, 200, 200, 200)
    assert chassis_motion(int(MotionMode.FORWARD), 201, 0) == (0, 0, 0, 0)
    assert chassis_motion(7, 100, 0) == (0, 0, 0, 0)


@pytest.mark.parametrize('kind,duty,counts,mode,rpm,want', [
    (Kind.OPEN_LOOP, 0, 0, 1, 1, True),
    (Kind.OPEN_LOOP, 1, 0, 1, 1, False),
    (Kind.POSITION, 0, 0, 1, 0, True),
    (Kind.POSITION, 0, 5, 1, 0, False),
    (Kind.HEARTBEAT, 0, 0, 0, 0, True),
    (Kind.MOVE, 0, 0, 1, 100, False),
    (Kind.MOVE, 0, 0, 1, 0, True),
    (Kind.STOP, 0, 0, 0, 0, True),
])
def test_command_stop_target_table(kind, duty, counts, mode, rpm, want):
    assert _command_stop_target(int(kind), duty, counts, mode, rpm) is want


@pytest.mark.parametrize('kind,duty,counts,maxrpm,mode,rpm,want', [
    (Kind.OPEN_LOOP, 1400, 0, 0, int(MotionMode.FORWARD), 0, True),
    (Kind.OPEN_LOOP, 1401, 0, 0, int(MotionMode.FORWARD), 0, False),
    (Kind.OPEN_LOOP, 500, 0, 0, int(MotionMode.TURN_LEFT), 0, False),
    (Kind.POSITION, 0, 33, 8, int(MotionMode.FORWARD), 0, True),
    (Kind.POSITION, 0, 33, 7, int(MotionMode.FORWARD), 0, False),
    (Kind.POSITION, 0, 33, 41, int(MotionMode.FORWARD), 0, False),
    (Kind.POSITION, 0, 33, 25, int(MotionMode.TURN_LEFT), 0, False),
    (Kind.HEARTBEAT, 0, 0, 0, 0, 0, True),
    # ★ 没有 else 兜底的那些 kind 落到最后一条判据上，必须仍然为真，
    #   否则 ARM 会当场把底盘锁死
    (Kind.ARM, 0, 0, 0, 0, 0, True),
    (Kind.STOP, 0, 0, 0, 0, 0, True),
    (Kind.DISARM, 0, 0, 0, 0, 0, True),
    (Kind.MOVE, 0, 0, 0, int(MotionMode.FORWARD), 201, False),
])
def test_command_target_valid_table(kind, duty, counts, maxrpm, mode, rpm, want):
    assert _command_target_valid(int(kind), duty, counts, maxrpm, mode, rpm,
                                 FirmwareParams()) is want


@pytest.mark.parametrize('kind,want', [
    (Kind.STOP, True), (Kind.MOVE, True), (Kind.DISARM, True),
    (Kind.OPEN_LOOP, True),
    # POSITION 不在中止列表里 —— busy 期间再发 POSITION 是空操作
    (Kind.POSITION, False), (Kind.HEARTBEAT, False), (Kind.ARM, False),
])
def test_position_abort_table(kind, want):
    assert _position_abort({'kind': int(kind)}, FirmwareParams()) is want


# ============================================================================
# 十一、与厂商实现的一致性（在真实运行时序下）
# ============================================================================

def test_vendor_chassis_control_agrees_with_the_model():
    """用**厂商自己的** `chassis_control.py` 编帧，喂给我们的固件模型。

    前面所有测试的帧都是我们自己编的；这一条是把厂商实现当"甲方"用，
    证明"我们理解的协议"和"厂商发出来的字节"在**固件视角**下是同一样东西。
    交接包不在时自动跳过（板子上没有那个目录）。
    """
    handoff = os.environ.get(
        'ACG720_HANDOFF_DIR',
        '/mnt/c/Users/ZZW/Desktop/robot_car_handoff_20261006')
    tools = os.path.join(
        handoff, 'motor', 'wifi_car_position_loop_baseline_20261006', 'tools')
    if not os.path.isfile(os.path.join(tools, 'chassis_control.py')):
        pytest.skip('交接包不在，跳过厂商交叉核对')

    sys.path.insert(0, tools)
    import chassis_control  # noqa: E402

    rig = Rig()
    seq = 0
    for kind, payload in ((chassis_control.TYPE_ARM, b''),
                          (chassis_control.TYPE_MOVE,
                           struct.pack('<BHB', 1, 100, 128)),
                          (chassis_control.TYPE_HEARTBEAT, b''),
                          (chassis_control.TYPE_STOP, b'')):
        rig.send_raw(chassis_control.encode_frame(kind, seq, payload))
        rig.advance()
        seq = (seq + 1) & 0xFF
    assert rig.fw.rx.crc_errors == 0
    assert rig.fw.rx.frame_errors == 0
    assert rig.fw.rx.duplicates == 0
    assert rig.fw.state != SafetyState.FAULT
    assert rig.ack_count() == 4
