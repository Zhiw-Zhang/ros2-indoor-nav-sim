#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ACG720 串口协议 V1 的一致性测试。

这个文件要钉住的东西按重要性排序：

  1. **CRC 算法**：必须是 CRC-16/CCITT-FALSE（poly 0x1021、init 0xFFFF、
     不反射、不异或输出）。自检值 `crc16(b"123456789") == 0x29B1`。
     我们旧实现用的是 MODBUS（0x4B37）—— 差一个算法，真机一帧都收不了。
  2. **黄金帧**：7 条命令的完整字节，取自交接包主机侧权威实现
     `chassis_control.py:encode_frame()` 的真实输出。
  3. **固件的拒绝条件**：LEN>8、LEN 与 TYPE 不匹配、取值越界。
     这些**不是"发出去被忽略"，而是会把底盘锁死在 FAULT**
     （chassis_command_rx.v:240-247 → chassis_safety_fsm.v:89-90），
     所以必须在主机侧就拦住。
  4. **SEQ 规则**：严格前进 1..127；STOP 例外。
  5. **与厂商参考实现的活体交叉核对**：只要交接包在本机，就把它 import 进来
     随机对拍两边编解码。这条是"我们的复刻是否忠实"最强证据；
     交接包不在时自动跳过（本仓库不依赖包外文件）。

跑法::

    python3 -m pytest src/my_robot_description/test/test_protocol_v1.py -v
"""

from __future__ import annotations

import binascii
import glob
import importlib.util
import os
import random
import struct
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, "..", "src")
sys.path.insert(0, _SRC)

import acg720_protocol_v1 as P  # noqa: E402


# ============================================================================
# 1. CRC
# ============================================================================

def test_crc_selfcheck_value():
    """CRC-16/CCITT-FALSE 的标准自检值。0x29B1 是算法指纹，写错就拿到别的值。"""
    assert P.crc16(b"123456789") == 0x29B1


def test_crc_empty_is_init():
    """空数据应等于初值 0xFFFF（我们旧实现的初值也是这个，所以光看空值区分不了）。"""
    assert P.crc16(b"") == 0xFFFF


def test_crc_differs_from_modbus():
    """把"我们曾经用错的那个算法"钉在这里，防止有人改回去。

    MODBUS 是反射的（poly 0xA001 反着来），CCITT-FALSE 不反射；
    同一串数据两者结果必然不同。旧实现给的是 0x4B37。
    """
    def modbus(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        return crc

    assert modbus(b"123456789") == 0x4B37          # 旧实现
    assert P.crc16(b"123456789") != modbus(b"123456789")


def test_crc_matches_binascii_crc_hqx():
    """与标准库 `binascii.crc_hqx(data, 0xFFFF)` 对拍。

    这不是多此一举：交接包用的就是 `binascii.crc_hqx`，把我们的查表实现
    与一个独立实现对齐，才能保证"和厂商主机侧代码逐字节等价"。
    """
    rng = random.Random(0xAC6720)
    for _ in range(300):
        n = rng.randrange(0, 64)
        assert P.crc16(data := bytes(rng.randrange(256) for _ in range(n))) \
            == binascii.crc_hqx(data, 0xFFFF)


# ============================================================================
# 2. 黄金帧（取自 chassis_control.py 的真实输出）
# ============================================================================

#: (名称, kind, seq, payload, 期望整帧 hex)
GOLDEN_FRAMES = [
    ("STOP", P.Kind.STOP, 0, b"",
     "A55A0100000074F2"),
    ("ARM", P.Kind.ARM, 0, b"",
     "A55A0101000044C5"),
    ("MOVE fwd 100rpm ratio=128", P.Kind.MOVE, 7, struct.pack("<BHB", 1, 100, 128),
     "A55A0102070401640080DEF7"),
    ("OPEN_LOOP fwd duty=1000", P.Kind.OPEN_LOOP, 9, struct.pack("<BHB", 1, 1000, 0),
     "A55A0104090401E80300B85C"),
    ("POSITION fwd 25rpm 3232cnt", P.Kind.POSITION, 17,
     struct.pack("<BBIH", 1, 25, 3232, 0),
     "A55A010511080119A00C00000000C9ED"),
    ("HEARTBEAT", P.Kind.HEARTBEAT, 42, b"",
     "A55A01062A00F9A9"),
    ("DISARM", P.Kind.DISARM, 200, b"",
     "A55A0103C800D934"),
]


@pytest.mark.parametrize("name,kind,seq,payload,expected_hex",
                         GOLDEN_FRAMES, ids=[g[0] for g in GOLDEN_FRAMES])
def test_golden_frame_bytes(name, kind, seq, payload, expected_hex):
    raw = P.encode_frame(kind, seq, payload)
    assert raw.hex().upper() == expected_hex
    assert P.decode_frame(raw) == (int(kind), seq, payload)


def test_golden_frames_decode_back():
    """整帧里各字段的位置也要单独钉一遍，避免"整体对、字段错"的巧合。"""
    raw = P.encode_frame(P.Kind.POSITION, 17, struct.pack("<BBIH", 1, 25, 3232, 0))
    assert raw[0:2] == b"\xA5\x5A"       # 帧头
    assert raw[2] == 0x01                # VER
    assert raw[3] == 0x05                # KIND
    assert raw[4] == 17                  # SEQ
    assert raw[5] == 8                   # LEN
    assert P.crc16(raw[2:-2]) == struct.unpack("<H", raw[-2:])[0]


def test_crc_covers_ver_through_payload_not_header():
    """CRC 覆盖范围：含 VER..PAYLOAD，**不含**帧头。

    这是最容易搞错的一项：覆盖范围多两个字节（把 A5 5A 也算进去）时，
    CRC 值照样是"合法的 16 位数"，只是和固件算的不一样 —— 于是每一帧都被
    当成 CRC 错，然后**锁死底盘**。
    """
    payload = struct.pack("<BHB", 1, 100, 128)
    raw = P.encode_frame(P.Kind.MOVE, 7, payload)

    body = raw[2:-2]                      # VER..PAYLOAD
    assert body == bytes((0x01, 0x02, 0x07, 0x04)) + payload
    assert struct.unpack("<H", raw[-2:])[0] == P.crc16(body)
    # 反例：把帧头也算进去，值就不一样了
    assert P.crc16(raw[:-2]) != P.crc16(body)


# ============================================================================
# 3. 固件的拒绝条件 —— 必须在主机侧拦住
# ============================================================================

def test_encode_command_rejects_payload_over_firmware_limit():
    """固件的 LEN 上限是 8（chassis_command_rx.v:208），不是主机侧的 48。

    发超过 8 字节的载荷出去，固件判协议故障并**锁存 FAULT**，所以这里必须抛。
    """
    assert P.MAX_COMMAND_PAYLOAD == 8
    assert P.MAX_FRAME_PAYLOAD == 48
    with pytest.raises(P.ProtocolError, match="固件上限"):
        P.encode_command(P.Kind.MOVE, 1, b"\x00" * 9)
    # 16 字节是提案里 0x85 那种长度（当前固件必拒），**先**撞到长度上限
    with pytest.raises(P.ProtocolError, match="固件上限"):
        P.encode_command(P.Kind.POSITION, 1, b"\x00" * 16)


@pytest.mark.parametrize("kind,length", [
    (P.Kind.STOP, 1), (P.Kind.ARM, 4), (P.Kind.MOVE, 0),
    (P.Kind.DISARM, 2), (P.Kind.OPEN_LOOP, 8), (P.Kind.POSITION, 4),
    (P.Kind.HEARTBEAT, 1),
])
def test_encode_command_enforces_per_kind_length(kind, length):
    """LEN 与 TYPE 必须精确匹配（chassis_command_rx.v:97-112），否则锁死底盘。"""
    with pytest.raises(P.ProtocolError, match="要求 payload"):
        P.encode_command(kind, 1, b"\x00" * length)


@pytest.mark.parametrize("kind", [0x07, 0x08, 0x10, 0x81, 0x84, 0xFF])
def test_encode_command_rejects_non_command_kinds(kind):
    """0x07 VELOCITY 是**未分配**的提案（INTERFACE.md），固件不认。

    0x08 及以上永远过不了语义校验（`command_kind` 只取低 3 位）。
    这些发出去都会锁死底盘，所以编码期就拦。
    """
    with pytest.raises((P.ProtocolError, ValueError)):
        P.encode_command(kind, 1, b"\x00" * 4)


def test_move_validation_matches_firmware():
    """0x02 的取值范围校验（chassis_command_rx.v:98-99）。"""
    P.pack_move(P.MotionMode.FORWARD, 200, 255)          # 边界允许
    P.pack_move(P.MotionMode.TURN_LEFT, 0, 0)            # mode 3/4 固件是允许的
    with pytest.raises(P.ProtocolError, match="mode"):
        P.pack_move(7, 10, 255)
    with pytest.raises(P.ProtocolError, match="base_rpm"):
        P.pack_move(P.MotionMode.FORWARD, 201, 255)
    with pytest.raises(P.ProtocolError, match="ratio"):
        P.pack_move(P.MotionMode.FORWARD, 10, 256)


def test_open_loop_validation_matches_firmware():
    """0x04 只允许直线，duty ≤ 1400（chassis_command_rx.v:100-103）。"""
    P.pack_open_loop(P.MotionMode.FORWARD, 1400)
    with pytest.raises(P.ProtocolError, match="只接受"):
        P.pack_open_loop(P.MotionMode.ROTATE_LEFT, 100)
    with pytest.raises(P.ProtocolError, match="duty"):
        P.pack_open_loop(P.MotionMode.FORWARD, 1401)


def test_position_validation_matches_firmware():
    """0x05：mode ∈ {1,2,5,6}、8 ≤ max_rpm ≤ 40、counts > 32（严格大于）。"""
    P.pack_position(P.MotionMode.FORWARD, 8, 33)
    P.pack_position(P.MotionMode.ROTATE_LEFT, 40, 0xFFFFFFFF)
    with pytest.raises(P.ProtocolError, match="只接受"):
        P.pack_position(P.MotionMode.TURN_LEFT, 25, 1000)
    with pytest.raises(P.ProtocolError, match="max_rpm"):
        P.pack_position(P.MotionMode.FORWARD, 7, 1000)
    with pytest.raises(P.ProtocolError, match="max_rpm"):
        P.pack_position(P.MotionMode.FORWARD, 41, 1000)
    with pytest.raises(P.ProtocolError, match="必须 >"):
        P.pack_position(P.MotionMode.FORWARD, 25, 32)     # 32 本身不合法


# ============================================================================
# 4. 解析器：重同步
# ============================================================================

def test_parser_handles_pure_stream():
    frames = [P.encode_frame(k, i, pl) for i, (k, pl) in enumerate([
        (P.Kind.STOP, b""), (P.Kind.MOVE, struct.pack("<BHB", 1, 50, 200)),
        (P.Kind.HEARTBEAT, b"")])]
    parser = P.FrameParser()
    got = parser.feed(b"".join(frames))
    assert got == [(0x00, 0, b""), (0x02, 1, struct.pack("<BHB", 1, 50, 200)),
                   (0x06, 2, b"")]
    assert parser.crc_errors == 0 and parser.discarded_bytes == 0


def test_parser_resyncs_on_a5_a5_5a():
    """帧头前多一个 A5 时不能退回去 —— 真固件允许 A5 A5 5A。

    依据 chassis_command_rx.v:178-183：ST_HEADER_1 收到 A5 时**留在原位**。

    这里 `A5 A5 A5 5A ...` 里第一个完整的 `A5 5A` 出现在下标 2，所以丢掉的是
    **2** 个字节（不是 1）—— 解析结果仍与固件一致，只是重同步点不同。
    """
    frame = P.encode_frame(P.Kind.ARM, 3)
    parser = P.FrameParser()
    got = parser.feed(b"\xA5\xA5" + frame)
    assert got == [(0x01, 3, b"")]
    assert parser.discarded_bytes == 2


def test_parser_resyncs_after_garbage_and_bad_crc():
    good = P.encode_frame(P.Kind.STOP, 5)
    bad = bytearray(P.encode_frame(P.Kind.ARM, 6))
    bad[-1] ^= 0xFF                       # 破坏 CRC 高字节
    parser = P.FrameParser()
    got = parser.feed(b"\x00\x11\x22" + bytes(bad) + good)
    assert got == [(0x00, 5, b"")]        # 坏帧之后的帧还能救回来
    assert parser.crc_errors == 1
    assert parser.discarded_bytes >= 3


def test_parser_rejects_bad_version_and_bad_length():
    """VER≠1 或 LEN>48 计 header_errors，并逐字节重找帧头。"""
    parser = P.FrameParser()
    parser.feed(b"\xA5\x5A\x02\x00\x00\x00\x00\x00")       # VER=2
    assert parser.header_errors >= 1

    parser = P.FrameParser()
    parser.feed(b"\xA5\x5A\x01\x00\x00\xFF\x00\x00")       # LEN=255
    assert parser.header_errors >= 1


def test_parser_split_across_feeds_and_partial_tail():
    """串口是流：一帧可能被切成任意多段，尾巴上的半帧必须留着。"""
    frame = P.encode_frame(P.Kind.POSITION, 9, struct.pack("<BBIH", 5, 25, 500, 0))
    parser = P.FrameParser()
    got = []
    for byte in frame:
        got.extend(parser.feed(bytes([byte])))
    assert got == [(0x05, 9, struct.pack("<BBIH", 5, 25, 500, 0))]
    assert parser.discarded_bytes == 0

    # 只喂前半段：不能吐帧，也不能丢
    parser2 = P.FrameParser()
    assert parser2.feed(frame[:5]) == []
    assert parser2.feed(frame[5:]) == [(0x05, 9, struct.pack("<BBIH", 5, 25, 500, 0))]


def test_parser_reset_clears_partial_frame():
    """串口重连必须 reset，否则残留的半帧会污染重连后的第一帧。"""
    frame = P.encode_frame(P.Kind.STOP, 1)
    parser = P.FrameParser()
    parser.feed(frame[:4])
    parser.reset()
    assert parser.feed(frame[4:]) == []


# ============================================================================
# 5. SEQ 规则
# ============================================================================

@pytest.mark.parametrize("new,old,expected", [
    (1, 0, True), (2, 1, True), (127, 0, True), (0, 255, True),
    (0, 0, False),                       # 重复
    (5, 5, False),                       # 重复
    (0, 1, False),                       # 落后 255，被当旧帧
    (1, 128, False),                     # delta = 129 ≥ 128
    (128, 0, False),                     # delta = 128，正好不算"新"
])
def test_seq_is_new_matches_firmware_rule(new, old, expected):
    """固件判据：delta = seq - last，要求 delta != 0 且 delta < 128。"""
    assert P.seq_is_new(new, old) is expected


def test_stop_bypasses_sequence_check():
    """STOP 是唯一例外：序号不新也执行（chassis_command_rx.v:248）。"""
    assert P.stop_bypasses_sequence(P.Kind.STOP) is True
    for kind in (P.Kind.ARM, P.Kind.MOVE, P.Kind.HEARTBEAT, P.Kind.POSITION):
        assert P.stop_bypasses_sequence(kind) is False


def test_sequence_tracker_wraps_at_255():
    tracker = P.SequenceTracker(254)
    assert [tracker.take() for _ in range(4)] == [254, 255, 0, 1]


def test_sequence_tracker_resync_from_telemetry():
    """主机重启后必须按 0x81 的 last_command_seq 对齐，不能从 0 硬来。

    不对齐的后果是最恶心的一种：命令发出去了，固件判为重复/旧帧**静默丢弃**，
    不 ACK、不报错、也不加 fault —— 现象就是"车不动，日志干净"。
    """
    tracker = P.SequenceTracker(0)
    assert tracker.synced is False
    tracker.resync_from_telemetry(200)
    assert tracker.synced is True
    assert tracker.sequence == 201
    assert [tracker.take() for _ in range(3)] == [201, 202, 203]


# ============================================================================
# 6. 命令载荷往返
# ============================================================================

def test_move_payload_roundtrip():
    payload = P.pack_move(P.MotionMode.TURN_RIGHT, 123, 77)
    assert len(payload) == 4
    assert payload == struct.pack("<BHB", 4, 123, 77)
    assert P.parse_move(payload) == {"motion_mode": 4, "base_rpm": 123,
                                     "turn_inner_ratio_q8": 77}


def test_position_payload_roundtrip():
    """0x05 是 8 字节 —— 旧实现写成了 9 字节的 `<ihBH`，多一个字节就全错。"""
    payload = P.pack_position(P.MotionMode.ROTATE_LEFT, 25, 1015)
    assert len(payload) == 8
    assert payload == struct.pack("<BBIH", 5, 25, 1015, 0)
    assert P.parse_position(payload) == {"motion_mode": 5, "max_rpm": 25,
                                        "target_counts": 1015, "reserved": 0}


def test_open_loop_payload_roundtrip():
    payload = P.pack_open_loop(P.MotionMode.BACKWARD, 1000)
    assert payload == struct.pack("<BHB", 2, 1000, 0)
    assert P.parse_open_loop(payload)["duty_cycles"] == 1000


def test_payload_length_guards():
    for fn, payload, size in [
        (P.parse_move, b"\x01\x02\x03", 4),
        (P.parse_open_loop, b"\x01\x02\x03\x04\x05", 4),
        (P.parse_position, b"\x00" * 7, 8),
    ]:
        with pytest.raises(P.ProtocolError, match="载荷应为"):
            fn(payload)


# ============================================================================
# 7. 遥测解码
# ============================================================================

def test_parse_ack():
    """ACK 载荷就是 2 字节：state + fault（chassis_telemetry_imu_tx.v:105-106）。"""
    payload = bytes((P.SafetyState.RUNNING, P.FaultReason.TIMEOUT))
    assert P.parse_ack(payload) == {"state": 2, "fault": 4}
    with pytest.raises(P.ProtocolError):
        P.parse_ack(b"\x00")


def test_parse_telemetry_layout():
    """0x81 是 38 字节；旧实现把它当成 34 字节的 0x84，字段全错位。"""
    payload = (
        bytes((P.SafetyState.RUNNING, P.FaultReason.NONE, 0x2A, 0x00))
        + struct.pack("<4h", 10, 20, 30, 40)          # target_rpm  A B C D
        + struct.pack("<4h", -10, -20, -30, -40)      # speed_rpm
        + struct.pack("<4H", 100, 200, 300, 400)      # duty_cycles
        + bytes((1, 2, 3, 4))                         # fault_abcd
        + struct.pack("<3H", 5, 6, 7)                 # crc/dup/frame errors
    )
    assert len(payload) == 38
    got = P.parse_telemetry(payload)
    assert got["state"] == P.SafetyState.RUNNING
    assert got["last_command_seq"] == 0x2A
    assert got["target_rpm_abcd"] == [10, 20, 30, 40]
    assert got["speed_rpm_abcd"] == [-10, -20, -30, -40]
    assert got["duty_cycles_abcd"] == [100, 200, 300, 400]
    assert got["fault_abcd"] == [1, 2, 3, 4]
    assert (got["crc_errors"], got["duplicates"], got["frame_errors"]) == (5, 6, 7)


def test_parse_diagnostic_carries_build_id():
    """0x83 的 build_id 是版本检测的抓手（chassis_telemetry_imu_tx.v:40）。"""
    payload = struct.pack("<HI I 4i 4h 4H",
                          0x1234, 0x01020304, 0x20260A02,
                          100, -200, 300, -400,
                          10, -20, 30, -40,
                          1, 2, 3, 4)
    assert len(payload) == 42
    got = P.parse_diagnostic(payload)
    assert got["diagnostic_counter"] == 0x1234
    assert got["fpga_uptime_ms"] == 0x01020304
    assert got["build_id"] == 0x20260A02
    assert got["build_id_hex"] == "20260A02"
    assert got["position_count_abcd"] == [100, -200, 300, -400]
    assert got["sample_delta_abcd"] == [10, -20, 30, -40]
    assert got["invalid_transition_count_abcd"] == [1, 2, 3, 4]


def test_parse_position_telemetry_bitfields():
    """0x84 是 16 字节，且 flags/meta 是位域（不是两个整数字段）。"""
    flags = (1 << 1) | (1 << 2)                    # done=1, result=OK(1)
    meta = (int(P.MotionMode.ROTATE_LEFT) << 3) | int(P.PositionState.DONE)
    payload = bytes((flags, meta)) + struct.pack("<HIiI", 0, 1200, 1180, 20)
    assert len(payload) == 16
    got = P.parse_position_telemetry(payload)
    assert got["busy"] is False and got["done"] is True
    assert got["result_code"] == P.PositionResult.OK
    assert got["state"] == P.PositionState.DONE
    assert got["motion_mode"] == P.MotionMode.ROTATE_LEFT
    assert got["remaining_counts"] == 20


def test_parse_imu_scaling():
    """0x82 是 36 字节。换算系数：±16 g / ±2000 dps / ±180°、温度 raw/100。"""
    payload = (bytes((0x0F, 0x53))                              # flags, last_frame_type
               + struct.pack("<H", 7)                            # sample_counter @2
               + struct.pack("<9h", 32768 // 2, 0, 0, 0, 0, 0, 0, 0, 0)   # @4..21
               + struct.pack("<h", 2500)                         # temperature @22
               + struct.pack("<H", 12)                           # age_ms @24
               + struct.pack("<3H", 1, 2, 3)                     # error counts @26..31
               + struct.pack("<I", 999))                         # uptime @32
    assert len(payload) == 36
    got = P.parse_imu(payload)
    assert got["sample_counter"] == 7
    assert got["last_frame_type"] == 0x53
    assert got["accel_g_xyz"][0] == pytest.approx(8.0)      # raw·16/32768
    assert got["temperature_raw"] == 2500
    assert got["temperature_c"] == pytest.approx(25.0)      # raw/100
    assert got["age_ms"] == 12
    assert got["fpga_uptime_ms"] == 999
    # flags=0x0F → accel/gyro/angle 有效 + fresh；stale/error 位都为 0
    assert got["fresh"] is True and got["stale"] is False
    assert (got["valid_accel"], got["valid_gyro"], got["valid_angle"]) == (True, True, True)


def test_decode_telemetry_frame_dispatch():
    payload = struct.pack("<HI I 4i 4h 4H", 0, 0, 0x20260A02,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    assert P.decode_telemetry_frame(P.Kind.DIAGNOSTIC, payload)["build_id"] == 0x20260A02
    with pytest.raises(P.ProtocolError, match="不是已知的遥测类型"):
        P.decode_telemetry_frame(P.Kind.MOVE, payload)


# ============================================================================
# 8. 里程计小工具
# ============================================================================

@pytest.mark.parametrize("new,old,expected", [
    (10, 5, 5), (-10, -5, -5),
    (-2147483648, 2147483647, 1),      # 正回绕
    (2147483647, -2147483648, -1),     # 负回绕
    (0, 0, 0),
])
def test_delta32_handles_wraparound(new, old, expected):
    """FPGA 的 position_count 是 s32 累计值，会回绕（core.py:delta32 同构）。"""
    assert P.delta32(new, old) == expected


def test_counts_to_metres_nominal():
    """1320 counts/圈、轮径 65 mm —— 与 position_control.py 的名义换算一致。

    厂商那边的单测写的是 `assertAlmostEqual(..., 6.4646, places=3)`，
    即容差 5e-4；这里用 abs=1e-3 更宽松一点，免得为了第 4 位小数打架。
    """
    import math
    counts_per_mm = 1320.0 / (math.pi * 65.0)
    assert counts_per_mm == pytest.approx(6.4646, abs=1e-3)
    assert P.counts_to_metres(1320.0, 1320.0, 0.0325) == pytest.approx(2 * math.pi * 0.0325)


# ============================================================================
# 9. 与厂商参考实现的活体交叉核对
# ============================================================================
# 交接包不在本仓库里，所以这条测试**没包就跳过**，绝不把仓库绑死在包外文件上。
# 但只要包在（开发机上就在），它就会跑，而且是最有说服力的一条：
# 拿厂商自己的 encode/decode/遥测解码跟我们的实现对拍。

def _find_vendor_module():
    """找交接包里的 chassis_control.py，找不到返回 None。"""
    candidates = []
    env = os.environ.get("ACG720_HANDOFF_DIR")
    if env:
        candidates.append(env)
    candidates += glob.glob("/mnt/c/Users/*/Desktop/robot_car_handoff_*")
    candidates += glob.glob(os.path.expanduser("~/robot_car_handoff_*"))
    for root in candidates:
        for hit in glob.glob(os.path.join(
                root, "motor", "*position_loop*", "tools", "chassis_control.py")):
            spec = importlib.util.spec_from_file_location("_vendor_chassis", hit)
            if spec is None or spec.loader is None:
                continue
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module, hit
    return None


_VENDOR = _find_vendor_module()


@pytest.mark.skipif(_VENDOR is None,
                    reason="交接包不在本机（设 ACG720_HANDOFF_DIR 可指定路径）")
def test_live_crosscheck_frames_against_vendor():
    vendor, path = _VENDOR
    rng = random.Random(20260)
    command_kinds = [int(k) for k in P.COMMAND_LENGTHS]
    for _ in range(400):
        kind = rng.choice(command_kinds + [0x80, 0x81, 0x82, 0x83, 0x84])
        seq = rng.randrange(256)
        length = rng.choice([0, 2, 4, 8, 16, 36, 38, 42, 48])
        payload = bytes(rng.randrange(256) for _ in range(length))
        ours = P.encode_frame(kind, seq, payload)
        theirs = vendor.encode_frame(kind, seq, payload)
        assert ours == theirs, f"kind=0x{kind:02X} seq={seq} len={length}"
        assert P.decode_frame(ours) == vendor.decode_frame(theirs)


@pytest.mark.skipif(_VENDOR is None,
                    reason="交接包不在本机（设 ACG720_HANDOFF_DIR 可指定路径）")
def test_live_crosscheck_crc_against_vendor():
    vendor, _ = _VENDOR
    rng = random.Random(0xC7C)
    for _ in range(200):
        # 长度上限两边都是 48，超了双方都会抛，这里只比合法区间
        body = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 49)))
        assert P.crc16(body) == binascii.crc_hqx(body, 0xFFFF)
        assert P.encode_frame(0x00, 0, body) == vendor.encode_frame(0x00, 0, body)


@pytest.mark.skipif(_VENDOR is None,
                    reason="交接包不在本机（设 ACG720_HANDOFF_DIR 可指定路径）")
def test_live_crosscheck_telemetry_decoders_against_vendor():
    """四种遥测解码逐字段对拍（厂商的 `build_id` 是 hex 字符串，我们是 int）。"""
    vendor, _ = _VENDOR
    rng = random.Random(4242)

    def rand_payload(n):
        return bytes(rng.randrange(256) for _ in range(n))

    # 0x81：厂商的键与本模块的键同名，直接比
    payload = rand_payload(38)
    ours, theirs = P.parse_telemetry(payload), vendor.decode_telemetry(0, payload)
    for key in ("state", "fault", "last_command_seq", "stby",
                "target_rpm_abcd", "speed_rpm_abcd", "duty_cycles_abcd",
                "fault_abcd", "crc_errors", "duplicates", "frame_errors"):
        assert ours[key] == theirs[key], key

    # 0x83：厂商把 build_id 格式化成 8 位大写 hex，我们返回 int + hex 两种
    payload = rand_payload(42)
    ours, theirs = P.parse_diagnostic(payload), vendor.decode_diagnostic(0, payload)
    assert ours["build_id_hex"] == theirs["build_id"]
    assert ours["build_id"] == int(theirs["build_id"], 16)
    for key in ("diagnostic_counter", "fpga_uptime_ms", "position_count_abcd",
                "sample_delta_abcd", "invalid_transition_count_abcd"):
        assert ours[key] == theirs[key], key

    # 0x84
    payload = rand_payload(16)
    ours, theirs = (P.parse_position_telemetry(payload),
                    vendor.decode_position_telemetry(0, payload))
    for key in ("busy", "done", "result_code", "state", "motion_mode",
                "base_rpm", "target_counts", "progress_counts", "remaining_counts"):
        assert ours[key] == theirs[key], key

    # 0x82：厂商对浮点做了 6 位四舍五入，我们保留精度
    payload = rand_payload(36)
    ours, theirs = P.parse_imu(payload), vendor.decode_imu(0, payload)
    assert ours["sample_counter"] == theirs["sample_counter"]
    assert ours["last_frame_type"] == theirs["last_frame_type"]
    assert ours["temperature_raw"] == theirs["temperature_raw"]
    for key in ("accel_raw_xyz", "gyro_raw_xyz", "angle_raw_rpy"):
        assert ours[key] == theirs[key], key
    for key in ("accel_g_xyz", "gyro_dps_xyz", "angle_deg_rpy"):
        for mine, his in zip(ours[key], theirs[key]):
            assert mine == pytest.approx(his, abs=1e-6), key
    for key in ("valid_accel", "valid_gyro", "valid_angle", "fresh", "stale",
                "uart_error_seen", "checksum_error_seen", "frame_timeout_seen",
                "age_ms", "checksum_errors", "frame_timeouts",
                "uart_frame_errors", "fpga_uptime_ms"):
        assert ours[key] == theirs[key], key


@pytest.mark.skipif(_VENDOR is None,
                    reason="交接包不在本机（设 ACG720_HANDOFF_DIR 可指定路径）")
def test_live_crosscheck_parser_against_vendor_probe():
    """滚动解析器与交接包 chassis_wireless_probe.extract_frames 的行为对拍。"""
    vendor_root = os.path.dirname(_VENDOR[1])
    spec = importlib.util.spec_from_file_location(
        "_vendor_probe", os.path.join(vendor_root, "chassis_wireless_probe.py"))
    if spec is None or spec.loader is None:
        pytest.skip("没找到 chassis_wireless_probe.py")
    probe = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(probe)

    rng = random.Random(777)
    stream = bytearray()
    for _ in range(60):
        kind = rng.choice([0x00, 0x01, 0x02, 0x05, 0x06, 0x81, 0x83])
        length = {0x02: 4, 0x05: 8, 0x81: 38, 0x83: 42}.get(kind, 0)
        stream += P.encode_frame(kind, rng.randrange(256),
                                 bytes(rng.randrange(256) for _ in range(length)))
        stream += bytes(rng.randrange(256) for _ in range(rng.randrange(0, 4)))

    errors: dict[str, int] = {"discarded_bytes": 0, "header_errors": 0,
                              "crc_errors_host": 0}
    theirs = list(probe.extract_frames(bytearray(stream), errors))
    ours = P.FrameParser().feed(bytes(stream))
    assert ours == theirs
    assert errors["crc_errors_host"] == 0
