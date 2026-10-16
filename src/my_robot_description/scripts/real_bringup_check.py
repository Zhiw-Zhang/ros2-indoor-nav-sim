#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ACG720 真机接线体检 —— 默认**一个字节都不发**，只被动读遥测。

为什么需要它
============
接上 FPGA 之后，"车不动"有六种完全不同的根因，而且现场看到的现象几乎一样：

  1. TX/RX 没交叉、或没共地        -> 一个字节都收不到（或随机出错）
  2. 拔错串口 / 开了板载 console   -> 收到的是内核日志，不是遥测
  3. 波特率不对                    -> 收到一堆垃圾字节，帧头永远对不上
  4. 烧的是曲线模型那版位流        -> 遥测正常，但位流版本号不对，驱动会
                                      拒绝一切运动命令（fail-safe，不是故障）
  5. 底盘锁在 FAULT（ESTOP/TIMEOUT）-> 命令全部被忽略，遥测看起来"正常"
  6. 主机发的帧格式错（CRC/LEN）    -> 把底盘**永久锁死**，且之前毫无征兆

前四条本脚本直接判掉；第 5 条从 0x81 的 state/fault 读出来；第 6 条由
`--arm` 用一条**最安全**的命令（ARM，不产生任何运动）去验证下行链路。

安全性（为什么默认只读）
========================
FPGA 从复位起就以 20 Hz 广播遥测 —— `chassis_telemetry_imu_tx.v:230` 的
周期计数器**不受安全状态门控**，所以不接命令也能读到 0x81/0x82/0x83/0x84。
因此默认模式不往串口写任何东西，**零风险**：怎么跑都不会让车动、也不会锁死。

`--arm` 才会发帧，而且固定按 "ARM → 等 ACK → DISARM" 收尾：
  · 只发 ARM / HEARTBEAT / DISARM，**绝不发 MOVE / POSITION** —— 车轮不会转；
  · 退出前一定发 DISARM（DISARMED 状态不受 300 ms 命令超时约束，
    `chassis_safety_fsm.v:62,104`），所以正常退出不会留下 FAULT；
  · ⚠ 如果进程在 ARM 之后被 `kill -9`，底盘会在 300 ms 后锁 FAULT_TIMEOUT，
    恢复方式是**先松开 S0 再按 S4**（FAULT 自锁，没有其它出路）。

用法
====
    # 1) 只读体检（推荐先跑这个）
    ros2 run my_robot_description real_bringup_check.py --port /dev/acg720

    # 2) 连下行链路一起验（会 ARM 再 DISARM，轮子不动）
    ros2 run my_robot_description real_bringup_check.py --port /dev/acg720 --arm

    # 3) 没有硬件：拿虚拟底盘先看一遍"正确长什么样"
    python3 src/my_robot_description/src/acg720_simulator.py     # 打印 /dev/pts/N
    ros2 run my_robot_description real_bringup_check.py --port /dev/pts/N --arm

退出码：0 = 没有硬阻塞；1 = 有 ✗ 级别的阻塞项。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from collections import defaultdict

# 纯 Python 协议模块在 install 里与本脚本同目录（lib/my_robot_description/），
# 但直接跑源码树时它们在 ../src 下 —— 两处都试，两种调用方式都能用。
_HERE = os.path.dirname(os.path.abspath(__file__))
for _cand in (os.path.join(_HERE, os.pardir, 'src'), _HERE):
    if os.path.exists(os.path.join(_cand, 'acg720_protocol_v1.py')):
        sys.path.insert(0, os.path.abspath(_cand))
        break
else:  # pragma: no cover —— 只有安装坏了才会走到
    print('找不到 acg720_protocol_v1.py：install 不完整，先 colcon build',
          file=sys.stderr)
    sys.exit(2)

import serial  # noqa: E402

from acg720_protocol_v1 import (  # noqa: E402
    FrameParser,
    FaultReason,
    Kind,
    PositionResult,
    PositionState,
    SafetyState,
    SequenceTracker,
    decode_telemetry_frame,
    encode_command,
)

#: 交接包里位置环那版位流的版本号（chassis_telemetry_imu_tx.v:40）
EXPECT_BUILD_ID = 0x20260A02

#: 心跳（喂狗）周期：300 ms 超时，10 Hz 有 3 倍余量
HEARTBEAT_HZ = 10.0

#: 遥测是 20 Hz —— 每类周期包每 50 ms 一帧
EXPECT_TELEMETRY_HZ = 20.0

OK = '  ✓'
BAD = '  ✗'
WARN = '  ⚠'
INFO = '  ·'

_blockers: list[str] = []


def hard(msg: str) -> None:
    _blockers.append(msg)


def section(title: str) -> None:
    print(f'\n=== {title} ===')


def state_cn(value: int) -> str:
    try:
        return f'{SafetyState(value).name}({value})'
    except ValueError:
        return f'未知({value})'


def fault_cn(value: int) -> str:
    try:
        return f'{FaultReason(value).name}({value})'
    except ValueError:
        return f'未知({value})'


# ============================================================================
# 链路读取
# ============================================================================

class LinkStats:
    """被动积累的遥测状态（最近一次解码结果 + 各类帧计数）。"""

    def __init__(self) -> None:
        self.counts: dict[int, int] = defaultdict(int)
        self.acks: dict[int, dict] = {}
        self.build_id: int | None = None
        self.diagnostic: dict | None = None
        self.chassis: dict | None = None
        self.imu: dict | None = None
        self.position: dict | None = None
        self.unknown_kinds: dict[int, int] = defaultdict(int)
        self.decode_errors: list[str] = []

    def note(self, kind: int, seq: int, payload: bytes) -> None:
        self.counts[kind] += 1
        try:
            if kind == Kind.ACK:
                # 自己解，因为 ACK 的 seq 才是"哪条命令被接受了"的凭据
                self.acks[seq] = decode_telemetry_frame(Kind.ACK, payload)
                return
            decoded = decode_telemetry_frame(kind, payload)
        except Exception as exc:  # noqa: BLE001 —— 未知类型/长度不符都只记录
            if kind in (Kind.ACK, Kind.TELEMETRY, Kind.IMU,
                        Kind.DIAGNOSTIC, Kind.POSITION_TELEMETRY):
                self.decode_errors.append(f'0x{kind:02X}: {exc}')
            else:
                self.unknown_kinds[kind] += 1
            return

        if kind == Kind.TELEMETRY:
            self.chassis = decoded
        elif kind == Kind.IMU:
            self.imu = decoded
        elif kind == Kind.DIAGNOSTIC:
            self.build_id = decoded['build_id']
            self.diagnostic = decoded
        elif kind == Kind.POSITION_TELEMETRY:
            self.position = decoded


def listen(ser: serial.Serial, parser: FrameParser, stats: LinkStats,
           seconds: float, tracker: SequenceTracker | None = None,
           heartbeat: bool = False) -> None:
    """读 `seconds` 秒。heartbeat=True 时同时以 10 Hz 发 HEARTBEAT 喂狗。

    喂狗是必须的：ARMED 状态下 300 ms 没收到有效命令就锁 FAULT_TIMEOUT
    （`chassis_safety_fsm.v:104`），所以"保持 ARM 并观察"必须持续发帧。
    """
    deadline = time.monotonic() + seconds
    next_beat = time.monotonic()
    while True:
        now = time.monotonic()
        if now >= deadline:
            return
        if heartbeat and tracker is not None and now >= next_beat:
            send(ser, tracker, Kind.HEARTBEAT)
            next_beat = now + 1.0 / HEARTBEAT_HZ
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            for kind, seq, payload in parser.feed(chunk):
                stats.note(kind, seq, payload)


def send(ser: serial.Serial, tracker: SequenceTracker, kind: int,
         payload: bytes = b'') -> int:
    """发一条命令，返回它的 SEQ（ACK 会带着同一个 SEQ 回来）。"""
    seq = tracker.take()
    ser.write(encode_command(kind, seq, payload))
    return seq


def wait_ack(ser: serial.Serial, parser: FrameParser, stats: LinkStats,
             seq: int, timeout: float) -> bool:
    """等 SEQ 匹配的 ACK。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if seq in stats.acks:
            return True
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            for kind, fseq, payload in parser.feed(chunk):
                stats.note(kind, fseq, payload)
    return seq in stats.acks


# ============================================================================
# 各节检查
# ============================================================================

def check_open(port: str, baudrate: float) -> serial.Serial | None:
    section(f'1. 串口 {port} @ {int(baudrate)} 8N1')
    if not os.path.exists(port):
        print(f'{BAD} 设备节点不存在')
        hard(f'{port} 不存在')
        print(f'{INFO} 宿主上看： ls -l /dev/ttyUSB* /dev/ttyACM*')
        print(f'{INFO} 容器里没透传：要用 ACG720_DEVICE={port} ./docker/run.sh …')
        return None
    try:
        ser = serial.Serial(port, baudrate, timeout=0.02)
    except serial.SerialException as exc:
        print(f'{BAD} 打不开：{exc}')
        hard(f'{port} 打不开（权限或占用）')
        print(f'{INFO} Permission denied -> 设备节点的 gid 没进容器：'
              f'docker/run.sh 的 --group-add 那一段')
        print(f'{INFO} 被占用 -> 是否有别的进程已经打开了它（驱动/串口助手）')
        return None
    print(f'{OK} 打开成功')
    return ser


def check_passive(stats: LinkStats, parser: FrameParser, duration: float) -> bool:
    section(f'2. 被动监听 {duration:.0f} s（不往串口写任何东西）')
    if not stats.counts:
        print(f'{BAD} 一个字节的遥测都没解析出来')
        hard('收不到任何遥测帧')
        print(f'{INFO} 已收到的字节里：垃圾 {parser.discarded_bytes} 字节，'
              f'帧头/版本错 {parser.header_errors}，CRC 错 {parser.crc_errors}')
        if parser.discarded_bytes > 1000:
            print(f'{WARN} 收到大量无法成帧的字节 —— 波特率/端口/电平的问题：'
                  f'收到的可能是别的东西（内核 console 日志）')
        print(f'{INFO} 逐条查：')
        print(f'{INFO}   1) 转接板 TXD -> FPGA E17(rxd)、RXD -> C20(txd)，'
              f'**必须交叉**')
        print(f'{INFO}   2) GND 共地（没共地会随机出错，而不是干脆不通）')
        print(f'{INFO}   3) 电平跳线 3.3 V')
        print(f'{INFO}   4) FPGA 上电、位流已下载、LED 有反应')
        print(f'{INFO}   5) 波特率 115200（FPGA 侧 50 MHz/115200 整数分频）')
        return False

    per_kind = {
        Kind.TELEMETRY: '0x81 底盘遥测',
        Kind.IMU: '0x82 IMU 转发',
        Kind.DIAGNOSTIC: '0x83 编码器诊断',
        Kind.POSITION_TELEMETRY: '0x84 位置遥测',
        Kind.ACK: '0x80 ACK',
    }
    for kind, name in per_kind.items():
        n = stats.counts.get(kind, 0)
        if kind == Kind.ACK:
            print(f'{INFO} {name}: {n} 条'
                  f'{"" if n == 0 else "（收到 ACK 说明之前发过命令）"}')
            continue
        hz = n / duration
        mark = OK if hz >= EXPECT_TELEMETRY_HZ * 0.7 else WARN
        print(f'{mark} {name}: {n} 条（{hz:.1f} Hz，期望 ≈'
              f'{EXPECT_TELEMETRY_HZ:.0f} Hz）')
        if hz < EXPECT_TELEMETRY_HZ * 0.7:
            print(f'{INFO}   偏低：丢字节会直接吃掉遥测帧。看下面的 CRC/垃圾字节')

    if stats.unknown_kinds:
        print(f'{WARN} 收到未知 KIND：'
              f'{ {f"0x{k:02X}": v for k, v in stats.unknown_kinds.items()} }')
        print(f'{INFO} 多半是位流版本比这份驱动新，或串口上混进了别的东西')
    if stats.decode_errors:
        print(f'{WARN} 有 {len(stats.decode_errors)} 条遥测解不开（前 3 条）：')
        for e in stats.decode_errors[:3]:
            print(f'{INFO}   {e}')

    healthy = (parser.crc_errors == 0 and parser.header_errors == 0
               and parser.discarded_bytes == 0)
    if healthy:
        print(f'{OK} 帧层干净：CRC 错 0、帧头错 0、垃圾字节 0')
    else:
        print(f'{WARN} 帧层不干净：CRC 错 {parser.crc_errors}、'
              f'帧头/版本错 {parser.header_errors}、'
              f'垃圾字节 {parser.discarded_bytes}')
        print(f'{INFO} 偶发个位数可以继续看；成片出错先解决电气问题'
              f'（共地、屏蔽、和电机电源隔离）')

    # 0x81 的 crc_errors/frame_errors 是**固件侧**收到的坏帧数：
    # 它不为 0 说明"主机发过去的东西"有问题，方向相反，要分开看。
    if stats.chassis:
        fw_crc = stats.chassis['crc_errors']
        fw_frame = stats.chassis['frame_errors']
        if fw_crc or fw_frame:
            print(f'{WARN} 固件侧计数：CRC 错 {fw_crc}、帧错 {fw_frame}')
            print(f'{INFO} 这是**下行**（主机→FPGA）的错，与上面的上行错不同源')
        else:
            print(f'{OK} 固件侧计数干净（crc_errors=0 frame_errors=0）')
    return True


def check_build_id(stats: LinkStats, expected: int) -> None:
    section('3. 位流版本（0x83 build_id）')
    if stats.build_id is None:
        print(f'{WARN} 没收到 0x83 诊断帧，无法判断烧的是哪个位流')
        print(f'{INFO} 后面所有运动命令都无法确认安全性 —— 先解决第 2 节')
        return
    got = stats.build_id
    print(f'{INFO} build_id = 0x{got:08X}（期望 0x{expected:08X}）')
    if got == expected:
        print(f'{OK} 位流版本符合，位置环可用')
    else:
        print(f'{BAD} 位流版本**不符**')
        hard(f'build_id=0x{got:08X} 不是位置环版本')
        print(f'{INFO} 交接包里有两个 .fs，位流不是位置环那版。驱动会**拒绝'
              f'一切运动命令**（fail-safe，不是故障）。')
        print(f'{INFO} 想用 0x05 POSITION 分段导航，必须烧 position_loop 那版')


def check_chassis(stats: LinkStats) -> None:
    section('4. 底盘状态（0x81）')
    c = stats.chassis
    if c is None:
        print(f'{WARN} 没收到 0x81 底盘遥测')
        return
    state, fault = c['state'], c['fault']
    print(f'{INFO} state={state_cn(state)}  fault={fault_cn(fault)}  '
          f'stby={c["stby"]}')
    if state == SafetyState.FAULT:
        print(f'{BAD} 底盘锁在 FAULT —— 所有命令都会被忽略，且它是**自锁**的')
        hard(f'底盘处于 FAULT（{fault_cn(fault)}）')
        print(f'{INFO} 唯一出路：断电，或**先松开 S0 再按 S4**')
        if fault == FaultReason.ESTOP:
            print(f'{INFO} FAULT_ESTOP：急停按钮还按着，或者按下过一次'
                  f'（estop_seen 需要松开 S0 才清零）')
        elif fault == FaultReason.TIMEOUT:
            print(f'{INFO} FAULT_TIMEOUT：ARMED 状态下 300 ms 没收到有效命令。'
                  f'主机侧要持续喂狗（10 Hz 心跳）')
        elif fault == FaultReason.PROTOCOL:
            print(f'{INFO} FAULT_PROTOCOL：主机发过格式非法的帧'
                  f'（CRC/LEN/取值越界），或命令挤进一个 ACK 槽')
    else:
        print(f'{OK} 底盘没锁死')

    print(f'{INFO} 目标转速 A/B/C/D = {c["target_rpm_abcd"]}，'
          f'实测 = {c["speed_rpm_abcd"]}')
    print(f'{INFO} 编码器累计计数在第 5 节（0x83）看')


def check_encoders(stats: LinkStats) -> None:
    section('5. 编码器（0x83）')
    d = stats.diagnostic
    if d is None:
        print(f'{WARN} 没收到 0x83 诊断帧')
        return
    labels = ('A 右前', 'B 左前', 'C 左后', 'D 右后')
    counts = d['position_count_abcd']
    invalid = d['invalid_transition_count_abcd']
    deltas = d['sample_delta_abcd']
    print(f'{INFO} 四轮累计计数（s32，会回绕，不是绝对值）：')
    for i, name in enumerate(labels):
        print(f'{INFO}   {name}: {counts[i]:>12d}   '
              f'50 ms 增量 {deltas[i]:>6d}   正交异常 {invalid[i]}')
    # invalid_transition_count 是 FPGA 侧的正交解码异常计数：轮子不转时
    # 应当为 0 且不增长。它增长说明编码器线序/干扰/接触有问题 ——
    # 这种问题表现为"里程计偶尔跳一下"，在图上完全看不出来。
    if any(invalid):
        print(f'{WARN} 有正交解码异常计数 —— 编码器信号质量有问题')
        print(f'{INFO} 查：A/B 相线序、屏蔽、与电机电源的隔离、接插件接触')
    else:
        print(f'{OK} 正交解码无异常')

    if any(deltas) and not stats.chassis:
        print(f'{INFO} 有增量说明轮子在动（或有人在转轮子）')
    print(f'{INFO} 手动转某个轮子，看它对应的计数是否**单调**变化 ——')
    print(f'{INFO} 这是确认"哪个计数对应哪个轮子"最快的方法，'
          f'也是确认 A/B/C/D 编号没接反的唯一办法')
    print(f'{INFO} 连续看： ros2 topic echo /acg720/telemetry_raw '
          f'--truncate-length 100000')


def check_imu(stats: LinkStats) -> None:
    section('6. IMU（0x82）')
    imu = stats.imu
    if imu is None:
        print(f'{WARN} 没收到 0x82 —— FPGA 那路 IMU UART（imu_rxd = D16）'
              f'可能没接模块')
        return
    print(f'{INFO} flags=0x{imu["flags"]:02X}  '
          f'valid(accel/gyro/angle)={imu["valid_accel"]}/'
          f'{imu["valid_gyro"]}/{imu["valid_angle"]}  '
          f'fresh={imu["fresh"]} stale={imu["stale"]}')
    print(f'{INFO} age={imu["age_ms"]} ms，采样计数={imu["sample_counter"]}，'
          f'FPGA 运行 {imu["fpga_uptime_ms"] / 1000.0:.1f} s')
    print(f'{INFO} 角度(rpy)={[round(v, 2) for v in imu["angle_deg_rpy"]]}°，'
          f'温度={imu["temperature_c"]:.1f} ℃')
    if imu['valid_accel'] or imu['valid_gyro'] or imu['valid_angle']:
        print(f'{OK} IMU 有数据 —— 这块是装了的')
    else:
        # 没装时 stale / 超时那几位**必然是置着的**（FPGA 收不到那一路 UART
        # 就会记这些历史标志），所以这里不再往下重复报一遍，免得看着像两个故障。
        print(f'{WARN} 一个 valid 位都没置 —— IMU 没装/没接/型号不符')
        print(f'{INFO} 这一条不影响底盘运动与分段导航（位置环只用编码器），'
              f'只影响"有没有第二个航向源"')
        return
    if imu['uart_error_seen'] or imu['checksum_error_seen'] \
            or imu['frame_timeout_seen']:
        print(f'{WARN} IMU 那一路有 UART 错/校验错/超时（历史标志位）')
        print(f'{INFO} IMU 是**独立的一路 UART**（imu_rxd = D16），'
              f'线序/共地/波特率要单独查一遍')


def check_position(stats: LinkStats) -> None:
    section('7. 位置环遥测（0x84）')
    p = stats.position
    if p is None:
        print(f'{WARN} 没收到 0x84')
        return
    try:
        st = PositionState(p['state']).name
    except ValueError:
        st = f'未知({p["state"]})'
    try:
        res = PositionResult(p['result_code']).name
    except ValueError:
        res = f'未知({p["result_code"]})'
    print(f'{INFO} busy={p["busy"]} done={p["done"]} state={st} result={res}')
    print(f'{INFO} mode={p["motion_mode"]} rpm={p["base_rpm"]} '
          f'目标计数={p["target_counts"]} 已走={p["progress_counts"]}')
    if p['busy']:
        print(f'{WARN} 此刻位置环是 busy —— 已经有一段在走。'
              f'**位置段运行中不能中止**（start_valid && !busy），只能等 done')
    else:
        print(f'{OK} 位置环空闲（没有段在走）')


def check_downlink(ser: serial.Serial, parser: FrameParser, stats: LinkStats,
                   tracker: SequenceTracker, hold: float) -> None:
    section('8. 下行链路（--arm：只发 ARM/HEARTBEAT/DISARM，轮子不会转）')
    # ★ SEQ 必须先跟遥测对齐：主机重启后 SEQ 归零，固件会判成"落后"而
    #   **静默丢弃**（chassis_command_rx.v:93-95），现象是"发了没反应、日志无错"。
    if stats.chassis is None:
        print(f'{WARN} 没读到 0x81 的 last_command_seq，无法对齐 SEQ')
        print(f'{INFO} 首次通信时固件是"来者不拒"（!last_seq_valid），'
              f'所以直接发也能通；但重连之后必须对齐')
    else:
        tracker.resync_from_telemetry(stats.chassis['last_command_seq'])
        print(f'{INFO} SEQ 已对齐到固件的 last_command_seq='
              f'{stats.chassis["last_command_seq"]}')

    armed_ok = False
    try:
        seq = send(ser, tracker, Kind.ARM)
        print(f'{INFO} 已发 ARM（SEQ={seq}），等 ACK…')
        if not wait_ack(ser, parser, stats, seq, 1.0):
            print(f'{BAD} 1 s 内没等到 ARM 的 ACK')
            hard('ARM 没有 ACK —— 下行链路不通')
            print(f'{INFO} 遥测能上来但命令下不去，查：')
            print(f'{INFO}   1) 转接板 TXD 是否真的接到了 E17（rxd）——'
                  f'这一根接错就正好是"只读不能写"')
            print(f'{INFO}   2) 电平 3.3 V、共地')
            print(f'{INFO}   3) FPGA 是否在 FAULT（第 4 节）')
            return
        print(f'{OK} 收到 ACK：state={state_cn(stats.acks[seq]["state"])} '
              f'fault={fault_cn(stats.acks[seq]["fault"])}')
        armed_ok = True

        if hold > 0:
            print(f'{INFO} 保持 ARM {hold:.1f} s，以 {HEARTBEAT_HZ:.0f} Hz 发 '
                  f'HEARTBEAT 喂狗（不发任何运动命令）…')
            listen(ser, parser, stats, hold, tracker, heartbeat=True)
            c = stats.chassis or {}
            print(f'{INFO} 期间 state={state_cn(c.get("state", -1))} '
                  f'fault={fault_cn(c.get("fault", -1))}')
            if c.get('state') == SafetyState.FAULT:
                print(f'{BAD} 保持期间锁进了 FAULT')
                hard('保持 ARM 期间锁进 FAULT')
            else:
                print(f'{OK} 保持期间没有锁死 —— 喂狗节奏是对的')
    finally:
        # ★ 收尾必须是 DISARM：留着 ARMED_STOP 退出，300 ms 后就会锁
        #   FAULT_TIMEOUT。这里尽力而为，失败也不能抛出去盖掉上面的结论。
        if armed_ok:
            try:
                seq = send(ser, tracker, Kind.DISARM)
                if wait_ack(ser, parser, stats, seq, 1.0):
                    print(f'{OK} 已 DISARM（state='
                          f'{state_cn(stats.acks[seq]["state"])}）')
                else:
                    print(f'{WARN} DISARM 没等到 ACK —— 底盘可能还停在 ARMED，'
                          f'300 ms 后会锁 FAULT_TIMEOUT，'
                          f'恢复要"松 S0 再按 S4"')
            except Exception as exc:  # noqa: BLE001
                print(f'{WARN} DISARM 发送失败：{exc}')
                print(f'{WARN} 底盘可能还停在 ARMED —— 同上，注意 300 ms 超时')


def main() -> int:
    ap = argparse.ArgumentParser(
        description='ACG720 真机接线体检：默认只读，--arm 才发命令')
    ap.add_argument('--port', default='/dev/acg720', help='串口设备节点')
    ap.add_argument('--baudrate', type=int, default=115200)
    ap.add_argument('--duration', type=float, default=5.0,
                    help='被动监听时长（秒）')
    ap.add_argument('--expect-build-id', default=f'{EXPECT_BUILD_ID:08X}',
                    help='期望的位流 build_id（十六进制）')
    ap.add_argument('--arm', action='store_true',
                    help='额外验证下行链路（ARM→ACK→DISARM，不发运动命令）')
    ap.add_argument('--hold', type=float, default=0.0,
                    help='--arm 时保持 ARM 多少秒（期间 10 Hz 喂狗）')
    args = ap.parse_args()

    try:
        expected = int(args.expect_build_id, 16)
    except ValueError:
        print(f'--expect-build-id 不是十六进制：{args.expect_build_id}',
              file=sys.stderr)
        return 2

    print('=' * 68)
    print('ACG720 真机接线体检')
    print('  默认只读：不往串口写任何东西' if not args.arm
          else '  --arm：会发 ARM/HEARTBEAT/DISARM，**不发运动命令**')
    print('=' * 68)

    ser = check_open(args.port, args.baudrate)
    if ser is None:
        return _finish()

    parser = FrameParser()
    stats = LinkStats()
    tracker = SequenceTracker()
    try:
        # 被动段：先把遥测攒起来，后面几节都读这份数据
        listen(ser, parser, stats, args.duration)
        ok = check_passive(stats, parser, args.duration)
        if ok:
            check_build_id(stats, expected)
            check_chassis(stats)
            check_encoders(stats)
            check_imu(stats)
            check_position(stats)
            if args.arm:
                check_downlink(ser, parser, stats, tracker, args.hold)
    finally:
        ser.close()

    return _finish()


def _finish() -> int:
    print('\n' + '=' * 68)
    if _blockers:
        print(f'有 {len(_blockers)} 项硬阻塞：')
        for i, b in enumerate(_blockers, 1):
            print(f'  {i}. {b}')
        print('=' * 68)
        return 1
    print('接线这一段通了 ✓')
    print('下一步：')
    print('  1) 轮子架空，起底盘（position 模式不需要 allow_run_limit，'
          '那条只针对连续 MOVE）：')
    print('       ros2 launch my_robot_description robot.launch.py '
          'nav:=false control_mode:=position')
    print('  2) 手动发一个目标试方向：')
    print('       ros2 topic pub --once /acg720/goal '
          'geometry_msgs/msg/PoseStamped \\')
    print('         \'{header: {frame_id: "odom"}, '
          'pose: {position: {x: 0.3, y: 0.0}}}\'')
    print('  3) 标 wheel_separation_scale：real_robot_params.md 第 2 节')
    print('=' * 68)
    return 0


if __name__ == '__main__':
    sys.exit(main())
