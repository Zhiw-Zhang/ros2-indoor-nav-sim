#!/usr/bin/env python3
"""ACG720 协议层 + 运动学的单元测试。

这些都是纯函数测试，不需要 ROS、不需要硬件，所以可以在任何机器上秒级跑完：

    python3 -m pytest src/my_robot_description/test/ -v

为什么值得写：协议字节布局和运动学符号是"接上真车才发现"的两类 bug，
而真车调试的代价远高于这里写测试的代价。尤其是**符号约定**——
左右轮搞反了车会反向转弯，而日志上看起来一切正常。
"""

import math
import os
import struct
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from acg720_protocol import (  # noqa: E402
    Cmd, Flag, Frame, FrameParser, PositionDone, STATUS_SIZE, Status,
    TELEMETRY_FORMAT, Telemetry,
    crc16, encode, lr_to_wheels, pack_heartbeat, pack_position,
    pack_position_done, pack_status, pack_telemetry, pack_velocity,
    parse_position_done, parse_status, parse_telemetry, parse_velocity,
    wheels_to_lr,
)
from drive_kinematics import (  # noqa: E402
    Pose2D, SkidSteerGeometry, quaternion_from_yaw, wrap_angle,
)


# ==========================================================================
# CRC
# ==========================================================================

def test_crc16_modbus_golden_vector():
    """CRC-16/MODBUS 的标准测试向量：b"123456789" → 0x4B37。

    这是钉死 CRC 实现是否正确的唯一硬证据。如果 FPGA 侧用的是别的多项式
    （比如 CCITT 0x1021），这个测试会立刻告诉我们，而不是等到现场 CRC 全红。
    """
    assert crc16(b"123456789") == 0x4B37


def test_crc16_empty():
    assert crc16(b"") == 0xFFFF


def test_crc16_differs_on_single_bit():
    a, b = crc16(b"\x01\x02\x03"), crc16(b"\x01\x02\x02")
    assert a != b
    # 单比特差应该让 CRC 差很多位，不是只差 1
    assert bin(a ^ b).count("1") > 3


# ==========================================================================
# 帧编解码：往返
# ==========================================================================

@pytest.mark.parametrize("cmd,payload", [
    (Cmd.HEARTBEAT, pack_heartbeat(0)),
    (Cmd.HEARTBEAT, pack_heartbeat(255)),
    (Cmd.SET_VELOCITY, pack_velocity(0, 0, 0, 0)),
    (Cmd.SET_VELOCITY, pack_velocity(30, 40, 40, 30)),
    (Cmd.SET_VELOCITY, pack_velocity(-20, -20, 20, 20)),
    (Cmd.SET_VELOCITY, pack_velocity(32767, -32768, 0, 1)),
    (Cmd.POSITION, pack_position(1320, 15, 1, 240)),
    (Cmd.POSITION, pack_position(-2640, 8, 0, 240)),
    (Cmd.TELEMETRY, b""),
    (0x99, b"\x01\x02\x03"),
])
def test_frame_roundtrip(cmd, payload):
    parser = FrameParser()
    frames = parser.feed(encode(cmd, payload))
    assert len(frames) == 1
    assert frames[0].cmd == cmd
    assert frames[0].payload == payload


def test_encode_has_correct_header_and_crc():
    raw = encode(Cmd.HEARTBEAT, b"\x07")
    assert raw[:2] == b"\xA5\x5A"
    assert raw[2] == 0x06
    assert raw[3] == 0x01
    assert raw[4] == 0x07
    # CRC 覆盖 TYPE+LEN+PAYLOAD
    assert struct.unpack("<H", raw[-2:])[0] == crc16(raw[2:-2])


def test_encode_rejects_oversized_payload():
    with pytest.raises(ValueError):
        encode(Cmd.TELEMETRY, b"\x00" * 65)


# ==========================================================================
# 帧解析：流式重新同步（真串口一定会遇到的场景）
# ==========================================================================

def test_parser_handles_split_reads():
    """一次 read 只拿到半个帧，也要能拼起来。"""
    raw = encode(Cmd.SET_VELOCITY, pack_velocity(1, 2, 3, 4))
    parser = FrameParser()
    # 逐字节喂
    frames = []
    for i in range(len(raw)):
        frames.extend(parser.feed(raw[i:i + 1]))
    assert len(frames) == 1
    assert frames[0].cmd == Cmd.SET_VELOCITY


def test_parser_handles_multiple_frames_in_one_read():
    parser = FrameParser()
    blob = (encode(Cmd.HEARTBEAT, b"\x01") + encode(Cmd.HEARTBEAT, b"\x02")
            + encode(Cmd.STATUS, pack_status(Status(2, True, 0, 1234))))
    frames = parser.feed(blob)
    assert [f.cmd for f in frames] == [Cmd.HEARTBEAT, Cmd.HEARTBEAT, Cmd.STATUS]


def test_parser_drops_leading_garbage():
    parser = FrameParser()
    frames = parser.feed(b"\x00\x11\x22" + encode(Cmd.HEARTBEAT, b"\x05"))
    assert len(frames) == 1
    assert frames[0].payload == b"\x05"
    assert parser.dropped_bytes == 3


def test_parser_resyncs_after_crc_error():
    """CRC 坏帧之后必须能恢复，否则一次干扰就永久失步。"""
    good1 = encode(Cmd.HEARTBEAT, b"\x01")
    bad = bytearray(encode(Cmd.HEARTBEAT, b"\x02"))
    bad[-1] ^= 0xFF            # 破坏 CRC
    good2 = encode(Cmd.HEARTBEAT, b"\x03")

    parser = FrameParser()
    frames = parser.feed(bytes(bad) + good1 + good2)
    # 坏帧被丢，两个好帧都要解出来
    assert [f.payload for f in frames] == [b"\x01", b"\x03"]
    assert parser.crc_errors == 1


def test_parser_rejects_absurd_length():
    """长度字段非法时不能傻等 255 字节，要立刻重新同步。"""
    parser = FrameParser()
    frames = parser.feed(b"\xA5\x5A\x84\xFF" + encode(Cmd.HEARTBEAT, b"\x09"))
    assert len(frames) == 1
    assert frames[0].payload == b"\x09"


def test_parser_keeps_partial_header():
    """只有一个 0xA5 时要留住它，下一批数据的 0x5A 才能凑成帧头。"""
    parser = FrameParser()
    assert parser.feed(b"\xA5") == []
    assert parser.pending_bytes == 1
    frames = parser.feed(b"\x5A" + encode(Cmd.HEARTBEAT, b"\x01")[2:])
    assert len(frames) == 1


def test_parser_records_unknown_cmd():
    """FPGA 将来加了新上报类型，要能看出来而不是静默丢弃。"""
    parser = FrameParser()
    frames = parser.feed(encode(0x90, b"\xAA"))
    assert frames[0].cmd == 0x90
    assert parser.unknown_cmd == [0x90]


# ==========================================================================
# payload 编解码
# ==========================================================================

def test_velocity_payload_is_int16_little_endian():
    payload = pack_velocity(1, -1, 300, -300)
    assert payload == struct.pack("<4h", 1, -1, 300, -300)
    assert len(payload) == 8


def test_velocity_rejects_out_of_range():
    with pytest.raises(ValueError):
        pack_velocity(40000, 0, 0, 0)


def test_position_payload_layout():
    payload = pack_position(counts=-1320, rpm=15, mode=1, timeout_samples=240)
    assert payload == struct.pack("<ihBH", -1320, 15, 1, 240)


def test_telemetry_roundtrip():
    t = Telemetry(seq=12345, uptime_ms=67890,
                  counts=(10, -20, 30, -40), rpm=(7, 8, -9, -10),
                  duty=(0, 821, 870, 2700), flags=Flag.LOW_SPEED_WARN | Flag.STALL)
    parsed = parse_telemetry(pack_telemetry(t))
    assert parsed == t


def test_telemetry_rejects_wrong_length():
    with pytest.raises(ValueError):
        parse_telemetry(b"\x00" * 5)


def test_telemetry_flag_names():
    t = Telemetry(0, 0, (0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0),
                  Flag.ESTOP | Flag.OVERSPEED)
    assert set(t.flag_names()) == {"ESTOP", "OVERSPEED"}


def test_telemetry_left_right_averaging():
    """A=右前 D=右后 B=左前 C=左后。这个顺序搞错就是原地转反向。"""
    t = Telemetry(0, 0, (0,) * 4, rpm=(40, 10, 10, 40), duty=(0,) * 4, flags=0)
    assert t.right_rpm == 40.0
    assert t.left_rpm == 10.0


def test_status_roundtrip():
    s = Status(heartbeat_echo=7, accepted=False, reject_code=1, uptime_ms=999)
    assert parse_status(pack_status(s)) == s


def test_status_reject_names_cover_all_codes():
    from acg720_protocol import REJECT_NAMES
    assert set(REJECT_NAMES) == {0, 1, 2, 3, 4}


def test_position_done_roundtrip():
    p = PositionDone(mode=1, success=False, reason=2,
                     final_counts=-1320, elapsed_samples=240)
    parsed = parse_position_done(pack_position_done(p))
    assert parsed.mode == 1
    assert parsed.success is False
    assert parsed.reason == 2
    assert parsed.final_counts == -1320
    assert parsed.elapsed_samples == 240


# ==========================================================================
# payload 布局：把字节数和字段偏移钉死
# ==========================================================================

def test_payload_sizes_are_exact():
    """字节数必须精确。

    这个测试的存在理由：`struct` 会**静默插入对齐 padding**。
    最初 STATUS 写成 "<IBBI" 时，calcsize 是 10 而不是直觉上的 7——
    uint32 被对齐到 4 字节边界，中间多了 3 个字节。
    协议里出现隐式 padding，就等于"上位机和 FPGA 对不上"，
    而且换一种语言重写时 padding 规则还可能不同。
    所以这里把每个 payload 的确切字节数钉死。
    """
    from acg720_protocol import (POSITION_DONE_SIZE, STATUS_SIZE,
                                 TELEMETRY_SIZE, VELOCITY_SIZE)
    assert VELOCITY_SIZE == 8          # 4 × int16
    assert TELEMETRY_SIZE == 34        # 2×u32 + 4×i16 + 4×i16 + 4×u16 + u16
    assert STATUS_SIZE == 8            # 3×u8 + 1 显式 pad + u32
    assert POSITION_DONE_SIZE == 10


def test_telemetry_field_offsets():
    """遥测每个字段的偏移必须与文档一致（真机靠这个对齐）。"""
    assert TELEMETRY_FORMAT == "<II4h4h4HH"
    t = Telemetry(seq=0x11223344, uptime_ms=0x55667788,
                  counts=(1, 2, 3, 4), rpm=(5, 6, 7, 8),
                  duty=(9, 10, 11, 12), flags=0xBEEF)
    raw = pack_telemetry(t)
    assert len(raw) == 34
    assert struct.unpack_from("<I", raw, 0)[0] == 0x11223344    # seq @0
    assert struct.unpack_from("<I", raw, 4)[0] == 0x55667788    # uptime @4
    assert struct.unpack_from("<4h", raw, 8) == (1, 2, 3, 4)    # counts @8
    assert struct.unpack_from("<4h", raw, 16) == (5, 6, 7, 8)   # rpm @16
    assert struct.unpack_from("<4H", raw, 24) == (9, 10, 11, 12)  # duty @24
    assert struct.unpack_from("<H", raw, 32)[0] == 0xBEEF       # flags @32


def test_status_layout_is_explicit_and_c_aligned():
    """STATUS 必须是 8 字节，uptime 落在偏移 4，且**不依赖 struct 的隐式对齐**。

    为什么较真这个（都是真实踩过的）：
      * "<IBBI"   → 14 字节（隐式插 4 字节）
      * "<IBB3xI" → 13 字节（uptime 在偏移 6）
      * "<IBBxI"  → 11 字节（开头的 I 让整体带 4 字节对齐，x 之后又被插 padding）
      * "<4BI"    → 8 字节 ✓（把 pad 写成第 4 个 B，位置完全由字段决定）

    所以这里断言的不是"格式串里有没有 x"，而是**字节布局本身**：
    pad 用一个恒为 0 的 B 表示，uint32 落在 4 字节边界（与 C 默认对齐一致）。
    """
    from acg720_protocol import STATUS_FORMAT
    assert STATUS_FORMAT == "<4BI", (
        "STATUS_FORMAT 必须是 <4BI：用 x 做 padding 时，前面的 4 字节字段会带来"
        "隐式对齐，calcsize 不再是直觉值")
    assert STATUS_SIZE == 8
    raw = pack_status(Status(7, False, 1, 0xAABBCCDD))
    assert len(raw) == 8
    assert raw[0] == 7 and raw[1] == 0 and raw[2] == 1
    assert raw[3] == 0                        # 显式 padding 必须是 0
    assert struct.unpack_from("<I", raw, 4)[0] == 0xAABBCCDD   # uptime @4


def test_velocity_field_order_is_A_B_C_D():
    """速度 payload 的顺序必须是 A,B,C,D（右前/左前/左后/右后）。

    顺序错了 = 车按错误的轮子组合运动。这里用四个互不相同的值钉住顺序。
    """
    raw = pack_velocity(11, 22, 33, 44)
    assert struct.unpack("<4h", raw) == (11, 22, 33, 44)
    from acg720_protocol import parse_velocity
    assert parse_velocity(raw).rpm == (11, 22, 33, 44)


# ==========================================================================
# 四轮 ↔ 左右 的归一化
# ==========================================================================

def test_wheels_to_lr_matches_handoff_normalization():
    """交接包 4.4 节：前进=(A+B+C+D)/4，左旋=(A−B−C+D)/4。

    用纯前进 (v,0) 和纯左旋 (0,ω) 两个工况反解，验证本文件的左右定义
    与 FPGA 的归一化**数学等价**。
    """
    # 纯前进：四轮同速 → 左右相等
    left, right = wheels_to_lr(20, 20, 20, 20)
    assert left == right == 20
    # 纯左旋：B=C=+20（左轮正转），A=D=−20（右轮反转）
    left, right = wheels_to_lr(-20, 20, 20, -20)
    assert left == 20 and right == -20


def test_lr_to_wheels_is_inverse():
    for left, right in [(0, 0), (20, 20), (10, 40), (-5, 7.5)]:
        a, b, c, d = lr_to_wheels(left, right)
        assert wheels_to_lr(a, b, c, d) == pytest.approx((left, right))


def test_f1_acceptance_case_reproduces_handoff_numbers():
    """用交接包 F1 验收里真实出现过的一组目标值回归。

    右弯 40RPM 轮速比 192/256 → 目标 (A,B,C,D) = (30,40,40,30)。
    左右应变 30 / 40，且归一化前进/左旋要能对上。
    """
    a, b, c, d = 30, 40, 40, 30
    left, right = wheels_to_lr(a, b, c, d)
    assert (left, right) == (40.0, 30.0)
    forward = (a + b + c + d) / 4.0          # FIFO 归一化
    left_spin = (a - b - c + d) / 4.0
    assert forward == 35.0
    assert left_spin == -5.0                  # 负 = 右旋
    assert (left + right) / 2 == forward
    assert (left - right) / 2 == -left_spin   # 左旋为正，与 FPGA 符号相反


# ==========================================================================
# 运动学
# ==========================================================================

def test_geometry_defaults_match_handoff():
    g = SkidSteerGeometry()
    assert g.wheel_radius_m == 0.0325          # 65 mm 轮径
    assert g.wheel_separation_m == 0.200       # 200 mm 轮距
    assert g.wheelbase_m == 0.185              # 185 mm 轴距
    assert g.counts_per_rev == 1320.0
    assert g.scale == 1.0


def test_counts_per_meter():
    g = SkidSteerGeometry()
    circ = 2 * math.pi * 0.0325
    assert g.wheel_circumference_m == pytest.approx(circ)
    assert g.counts_per_meter == pytest.approx(1320.0 / circ)
    # 1320 counts / (2π·0.0325) ≈ 6464 counts/m
    assert g.counts_per_meter == pytest.approx(6464.0, rel=1e-3)


def test_rpm_per_mps():
    g = SkidSteerGeometry()
    # 60/(2π·0.0325) ≈ 293.9 RPM per m/s
    assert g.rpm_per_mps == pytest.approx(293.9, rel=1e-3)
    assert g.rpm_to_mps(g.mps_to_rpm(0.37)) == pytest.approx(0.37)


def test_twist_wheels_roundtrip_uses_effective_separation():
    """Twist → 轮速 → Twist 必须闭环，且 scale 生效。"""
    for scale in (0.8, 1.0, 1.36, 2.0):
        g = SkidSteerGeometry(scale=scale)
        left, right = g.twist_to_wheels(0.3, 0.5)
        v, omega = g.wheels_to_twist(left, right)
        assert v == pytest.approx(0.3)
        assert omega == pytest.approx(0.5)


def test_scale_1_36_needs_more_wheel_differential():
    """scale > 1 意味着"要转同样的角，轮子必须差得更多"，用来补偿侧滑。"""
    base = SkidSteerGeometry(scale=1.0)
    corr = SkidSteerGeometry(scale=1.36)
    l0, r0 = base.twist_to_wheels(0.0, 1.0)
    l1, r1 = corr.twist_to_wheels(0.0, 1.0)
    assert (r1 - l1) > (r0 - l0)
    assert (r1 - l1) == pytest.approx((r0 - l0) * 1.36)


def test_pure_forward_gives_equal_wheels():
    g = SkidSteerGeometry()
    left, right = g.twist_to_wheels(0.2, 0.0)
    assert left == pytest.approx(right)
    assert left == pytest.approx(0.2 * g.rpm_per_mps)


def test_pure_rotation_sign_convention():
    """左旋 ω>0 → 右轮快于左轮（ROS REP-103：z 轴向上，逆时针为正）。"""
    g = SkidSteerGeometry()
    left, right = g.twist_to_wheels(0.0, 1.0)
    assert right > left
    # 左旋时左轮应该是反转
    assert left < 0 < right


def test_counts_to_distance_and_rotation():
    g = SkidSteerGeometry()
    d = g.counts_to_distance_m(1320)
    assert d == pytest.approx(g.wheel_circumference_m)
    assert g.distance_to_counts(d) == pytest.approx(1320)

    # 右轮走一圈、左轮走一圈的负值 → 原地转，转角 = 2·(πD) / b_eff
    rot = g.counts_to_rotation_rad(-1320, 1320)
    expected = 2 * g.wheel_circumference_m / g.effective_separation_m
    assert rot == pytest.approx(expected)


def test_effective_separation_scales_rotation():
    """同样轮差，scale 越大报出的转角越小 —— 这就是里程计尺度的作用。"""
    raw = SkidSteerGeometry(scale=1.0).counts_to_rotation_rad(-1320, 1320)
    cal = SkidSteerGeometry(scale=1.36).counts_to_rotation_rad(-1320, 1320)
    assert cal == pytest.approx(raw / 1.36)


# ==========================================================================
# 位姿积分
# ==========================================================================

def test_pose_straight_line():
    p = Pose2D()
    for _ in range(20):            # 20 × 50 ms = 1 s
        p = p.integrate(1.0, 0.0, 0.05)
    assert p.x == pytest.approx(1.0)
    assert p.y == pytest.approx(0.0)
    assert p.yaw == pytest.approx(0.0)


def test_pose_full_circle_closes():
    """原地转 2π 必须回到原点、yaw 回 0 —— 中点积分应当几乎无漂移。"""
    omega = 2 * math.pi / 2.0      # 2 秒转一圈
    p = Pose2D()
    for _ in range(40):            # 2 s / 50 ms
        p = p.integrate(0.0, omega, 0.05)
    assert p.x == pytest.approx(0.0, abs=1e-12)
    assert p.y == pytest.approx(0.0, abs=1e-12)
    assert p.yaw == pytest.approx(0.0, abs=1e-9)


def test_pose_arc_start_and_end():
    """走一段圆弧：起点朝 +x，半径 R=v/ω，转约 π/2 后位置应到约 (R, R)。

    ⚠ 注意这里**不**断言 yaw 精确等于 π/2。采样周期 dt=0.05 s、ω=0.5 rad/s
    时，π/2 需要 15.71 个采样——不是整数。离散化误差是 **O(ω·dt) ≈ 0.021 rad**，
    这是格式本身的属性，不是 bug，所以这里分别钉住两件事：
      1. 转角精确等于 n·ω·dt（离散积分的解析真值）；
      2. 离散化误差确实被 ω·dt 这个量级界住（证明没有实现错误）。
    """
    v, omega, dt = 0.3, 0.5, 0.05
    R = v / omega

    n = int(round((math.pi / 2 / omega) / dt))      # 16 个采样 ≈ 0.8 s
    p = Pose2D()
    for _ in range(n):
        p = p.integrate(v, omega, dt)

    # 1) 离散真值：每个采样精确加 ω·dt
    assert p.yaw == pytest.approx(n * omega * dt, abs=1e-12)
    # 2) 离散化误差在 ω·dt 量级内
    assert abs(p.yaw - math.pi / 2) <= omega * dt
    # 3) 终点落在半径 R 的圆弧上。
    #    左转的瞬时圆心在车的**左侧**，即 (0, R)，不是 (R, R) —— 起点在圆上，
    #    终点 (R,R) 也满足 x² + (y−R)² = R²，所以两者到圆心的距离都应是 R。
    assert math.hypot(p.x - 0.0, p.y - R) == pytest.approx(R, rel=1e-3)
    assert p.x == pytest.approx(R * math.sin(p.yaw), rel=1e-3)
    assert p.y == pytest.approx(R * (1 - math.cos(p.yaw)), rel=1e-3)


def test_pose_yaw_rate_is_exact_regardless_of_dt():
    """ω 是常量时，转角必须精确等于 ω·T，与 dt 怎么分无关。"""
    omega, T = 0.7, 2.3
    for dt in (0.05, 0.02, 0.11):
        p = Pose2D()
        n = int(T / dt)
        for _ in range(n):
            p = p.integrate(0.0, omega, dt)
        assert p.yaw == pytest.approx(wrap_angle(omega * n * dt), abs=1e-12)


def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-3 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(2 * math.pi) == pytest.approx(0.0, abs=1e-12)


def test_quaternion_from_yaw():
    x, y, z, w = quaternion_from_yaw(math.pi / 2)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sin(math.pi / 4))
    assert w == pytest.approx(math.cos(math.pi / 4))
    assert z * z + w * w == pytest.approx(1.0)
