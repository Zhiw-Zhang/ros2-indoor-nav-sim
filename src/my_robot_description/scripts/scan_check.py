#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2D 雷达（/scan）就绪自检 —— 只读订阅，不发布任何命令。

为什么需要它
============
雷达装好那一刻，能出的问题全都长得像"导航不动"，但根因完全在不同的层：

  1. 驱动没起来 / 话题名不是 /scan      -> 订阅不到，Nav2 一直等
  2. frame_id 没人发对应的 TF           -> costmap 报 "Timed out waiting for
                                           transform"，但 /scan 看起来"有数据"
  3. 驱动用 RELIABLE 发、costmap 用      -> 数据在，但订阅端一条都收不到
     BEST_EFFORT 收（或反过来）
  4. 自遮挡（车体挡住自己一圈）          -> costmap 里永久趴着一个幽灵障碍，
                                           车原地转圈、报 "no valid path"
  5. 无回波表示法不统一（inf / NaN /    -> 有的驱动发 inf，有的发 range_max，
     等于 range_max）                      滤波和 costmap 的判据跟着不一样
  6. 时间戳用的是 sim time 而节点用      -> 扫描被判成过期数据丢掉
     的是 wall time（或反过来）

这六条里有五条**看不出来**——话题在、有数据、日志无错。所以这份自检不是
"看一眼能不能收到"，而是逐条把上面六类钉住，让一次上电就有结论。

用法
====
    ros2 run my_robot_description scan_check.py
    ros2 run my_robot_description scan_check.py --topic /scan --duration 10
    ros2 run my_robot_description scan_check.py -q      # 只看有问题的项

    没有雷达时也能验这个脚本本身：先在仿真里跑（Gazebo 的 gpu_lidar 就发
    /scan），看到一个全绿的基线，再去接真雷达。

退出码：0 = 没有硬阻塞；1 = 有 ✗ 级别的阻塞项（雷达这段现在还不能用）。

【判据来源】角范围/点数/无回波占比这些**没有绝对标准**，取决于型号与安装，
所以本脚本只报实测值 + 与"能不能用于 SLAM/代价地图"的经验阈值对比，
最终判断留给人。硬阻塞只有一条：**收不到任何 /scan**。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# ---- 与工程其它部分约定的名字 -------------------------------------------------
# 这两个名字写死在 URDF / RViz / 文档里，改名会连锁改配置，所以在这里显式
# 声明成"契约"，而不是散落在判断里。
EXPECT_TOPIC = '/scan'
EXPECT_FRAME = 'laser_frame'
BASE_FRAME = 'base_link'

OK = '  ✓'
BAD = '  ✗'
WARN = '  ⚠'
INFO = '  ·'

#: 一个 10° 扇区里超过这个比例是"无回波"，就判成疑似遮挡/盲区
OCCLUSION_RATIO = 0.90

#: 发布频率低于它就明显不够用（Nav2 的 costmap 与 AMCL 都吃 10 Hz 左右）
MIN_RATE_HZ = 5.0

_blockers: list[str] = []


def hard(msg: str) -> None:
    """记一条硬阻塞（✗）。"""
    _blockers.append(msg)


def section(title: str) -> None:
    print(f'\n=== {title} ===')


# ============================================================================
# 纯函数部分：不依赖 ROS，可单独喂数据测（见 --selftest）
# ============================================================================

def analyse_scan(ranges, angle_min: float, angle_increment: float,
                 range_min: float, range_max: float,
                 bins: int = 36) -> dict:
    """统计一帧扫描。返回各类计数与疑似盲区扇区。

    无回波的三种表示法（NaN / inf / 恰好等于 range_max）在这里统一归类，
    因为不同驱动的表示法不同，而下游关心的是同一件事："这个方向看不到东西"。
    """
    total = len(ranges)
    nan_or_inf = 0
    below_min = 0
    no_return = 0
    valid_dist: list[float] = []
    bin_total = [0] * bins
    bin_blind = [0] * bins
    # 无回波判据的容差：有的驱动给的正是 range_max，有的是 range_max - 1e-3
    no_return_floor = max(range_max - 1e-3, range_max * 0.999)

    for i, r in enumerate(ranges):
        # ★ 扇区按**采样下标**分，不按角度算。
        #   LaserScan 的 angle_increment 是均匀的，所以下标就是角度的精确
        #   刻度；反过来用 `(angle-angle_min)/span*bins` 算，会在每个扇区
        #   边界上被浮点误差推错一个点（例如名义 18.000 的边界算成
        #   17.999999996），把一整片盲区报成锯齿状的 "100% / 90%"。
        b = min(i * bins // total, bins - 1)
        bin_total[b] += 1

        if r is None or math.isnan(r) or math.isinf(r):
            nan_or_inf += 1
            bin_blind[b] += 1
            continue
        if r < range_min:
            below_min += 1
            bin_blind[b] += 1
            continue
        if r >= no_return_floor:
            no_return += 1
            bin_blind[b] += 1
            continue
        valid_dist.append(r)

    blind_sectors: list[tuple[float, float, float]] = []
    for b in range(bins):
        if bin_total[b] == 0:
            continue
        ratio = bin_blind[b] / bin_total[b]
        if ratio >= OCCLUSION_RATIO:
            # 扇区的角度范围按它覆盖的**下标区间**折算，不硬编码 360/bins：
            #  部分扫描（比如 270° 的型号）下两者不相等。
            i0 = b * total // bins
            i1 = min((b + 1) * total // bins, total)
            start = math.degrees(angle_min + i0 * angle_increment)
            end = math.degrees(angle_min + i1 * angle_increment)
            blind_sectors.append((start, end, ratio))

    return {
        'total': total,
        'valid': len(valid_dist),
        'nan_or_inf': nan_or_inf,
        'below_min': below_min,
        'no_return': no_return,
        'blind_sectors': blind_sectors,
        'min_valid': min(valid_dist) if valid_dist else None,
        'max_valid': max(valid_dist) if valid_dist else None,
        'valid_ratio': (len(valid_dist) / total) if total else 0.0,
    }


def _selftest() -> int:
    """不给 ROS 也不给硬件，用构造数据验证统计逻辑。

    三个构造场景：正常一圈、带一块 90° 盲区、以及全 invalid。
    """
    print('analyse_scan 自检（不连 ROS、不连硬件）')
    n = 360
    inc = 2.0 * math.pi / n

    good = analyse_scan([3.0] * n, -math.pi, inc, 0.05, 12.0)
    assert good['valid'] == n and not good['blind_sectors'], good

    # 90° 盲区：第 90..179 个点无回波
    ranges = [3.0] * n
    for i in range(90, 180):
        ranges[i] = float('inf')
    occl = analyse_scan(ranges, -math.pi, inc, 0.05, 12.0)
    # inf 归到 nan_or_inf 里（和 NaN 一样都表示"这个方向没东西"），只有
    # **恰好等于 range_max** 才算 no_return —— 三种表示法分开统计，这样报告
    # 里能看出驱动用的是哪一种（不同驱动不一样，判据要跟着走）。
    assert occl['nan_or_inf'] == 90 and occl['no_return'] == 0, occl
    assert len(occl['blind_sectors']) == 9, occl['blind_sectors']

    dead = analyse_scan([float('nan')] * n, -math.pi, inc, 0.05, 12.0)
    assert dead['valid'] == 0 and len(dead['blind_sectors']) == 36, dead

    print(f'{OK} 正常一圈  -> valid={good["valid"]} 盲区=0')
    print(f'{OK} 90° 盲区  -> inf={occl["nan_or_inf"]} '
          f'盲区扇区={len(occl["blind_sectors"])}')
    print(f'{OK} 全无效    -> valid=0 盲区扇区={len(dead["blind_sectors"])}')
    print('自检通过 ✓（这些数字是构造的，不代表任何真实雷达）')
    return 0


# ============================================================================
# ROS 部分
# ============================================================================

class ScanProbe(Node):
    """只订阅 /scan 与 /tf，不发布任何东西。"""

    def __init__(self, topic: str) -> None:
        super().__init__('scan_check')
        self.msgs: list[LaserScan] = []
        # ★ 用 sensor QoS（BEST_EFFORT）。这是关键的一处：
        #   BEST_EFFORT 订阅者**既能收** BEST_EFFORT 也能收 RELIABLE 的发布者；
        #   反过来（RELIABLE 订阅 + BEST_EFFORT 发布）则一条都收不到。
        #   所以自检用 sensor QoS 才不会被 QoS 不匹配骗过去。
        self.create_subscription(LaserScan, topic, self._on_scan,
                                 qos_profile_sensor_data)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publishers_seen = 0

    def _on_scan(self, msg: LaserScan) -> None:
        self.msgs.append(msg)

    def spin_for(self, seconds: float, stop_after_first: bool = False) -> None:
        """转圈收消息，直到超时（或收到第一帧就停）。"""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if stop_after_first and self.msgs:
                return


def check_topic(probe: ScanProbe, topic: str, timeout: float) -> bool:
    section(f'1. 话题 {topic}')
    print(f'{INFO} 用 sensor QoS(BEST_EFFORT) 订阅，等第一帧（最多 {timeout:.0f} s）…')
    probe.spin_for(timeout, stop_after_first=True)
    # ★ 发布者数量要**等完之后**再查：DDS 发现是异步的，刚起节点那一刻
    #   查出来经常是 0，于是"没有发布者"和"收到第一帧"会一起打印出来，
    #   自相矛盾，看的人第一反应是"这脚本坏了"。
    publishers = probe.count_publishers(topic)
    if probe.msgs:
        print(f'{OK} 收到第一帧（话题上现在有 {publishers} 个发布者）')
        return True

    print(f'{BAD} {timeout:.0f} s 内一帧都没收到')
    hard(f'{topic} 收不到任何数据')
    if publishers == 0:
        print(f'{INFO} 而且话题上**没有**发布者 —— 数据源就没在发。按顺序查：')
        print(f'{INFO}   1) 驱动起来了吗：      ros2 node list')
        print(f'{INFO}   2) 话题名对吗（未必叫 /scan）：'
              f' ros2 topic list | grep -i scan')
        print(f'{INFO}   3) 串口/USB 在不在、被别的进程占了吗')
    else:
        print(f'{WARN} 但话题上**有** {publishers} 个发布者 —— 说明驱动活着、'
              f'只是数据没到你手上：')
        print(f'{INFO}   1) QoS 不匹配：驱动发 RELIABLE 而你收 BEST_EFFORT 是能收的，')
        print(f'{INFO}      反过来（发的 BEST_EFFORT、收的 RELIABLE）才会一条都收不到。')
        print(f'{INFO}      查： ros2 topic info {topic} --verbose')
        print(f'{INFO}   2) 驱动卡在读串口（独占冲突、波特率不对、型号不符）——'
              f'看驱动自己的日志')
    return False


def check_metadata(msg: LaserScan, expect_frame: str) -> None:
    section('2. 消息元数据')
    n = len(msg.ranges)
    print(f'{INFO} frame_id   : {msg.header.frame_id!r}')
    if msg.header.frame_id == expect_frame:
        print(f'{OK} frame_id 与工程约定一致（{expect_frame}）')
    elif not msg.header.frame_id:
        print(f'{BAD} frame_id 是空的 —— 没有坐标系，costmap 无法把点变换到地图系')
        hard('scan.header.frame_id 为空')
    else:
        print(f'{WARN} frame_id 不是约定的 {expect_frame!r}，而是 '
              f'{msg.header.frame_id!r}')
        print(f'{INFO}   只要 TF 里真有一条 base_link -> '
              f'{msg.header.frame_id} 也能用；但 URDF、RViz 配置、')
        print(f'{INFO}   文档里写的都是 {expect_frame}，不一致会让后面每个人'
              f'都多花时间怀疑一次')

    print(f'{INFO} 角度范围   : [{math.degrees(msg.angle_min):.1f}°, '
          f'{math.degrees(msg.angle_max):.1f}°]，步长 '
          f'{math.degrees(msg.angle_increment):.3f}°')
    print(f'{INFO} 点数       : {n}')
    print(f'{INFO} 声明距离   : [{msg.range_min:.3f}, {msg.range_max:.3f}] m')
    print(f'{INFO} intensities: {"有" if msg.intensities else "无"}')
    print(f'{INFO} 扫描时间   : {msg.scan_time:.3f} s，'
          f'两次之间 {msg.time_increment:.6f} s')

    span = msg.angle_max - msg.angle_min
    if n < 2:
        print(f'{BAD} 点数 {n} 太小，不构成一帧扫描')
        hard(f'ranges 只有 {n} 个点')
    elif abs(span) < math.radians(180.0):
        print(f'{WARN} 角度跨度只有 {math.degrees(span):.0f}°(<180°)，'
              f'不是常见 360° 扫描 —— 确认这是型号特性还是被裁剪过')
    else:
        print(f'{OK} 角度跨度 {math.degrees(span):.0f}°，点数足够')
    if msg.range_max <= msg.range_min:
        print(f'{BAD} range_max <= range_min，参数不合法')
        hard('range_max <= range_min')


def check_tf(probe: ScanProbe, frame: str, base_frame: str) -> None:
    section('3. TF：雷达到车体')
    if not frame:
        print(f'{WARN} frame_id 为空，跳过 TF 检查')
        return
    # 等一会儿，TF 是异步的：刚起节点时可能还没到
    deadline = time.monotonic() + 3.0
    tf = None
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.1)
        try:
            tf = probe.buffer.lookup_transform(base_frame, frame,
                                               rclpy.time.Time())
            break
        except Exception:  # noqa: BLE001 —— tf2 抛的异常类型很杂，统一重试
            continue
    if tf is None:
        print(f'{BAD} {base_frame} -> {frame} 查不到')
        hard(f'TF {base_frame} -> {frame} 不存在')
        print(f'{INFO} 这会让 costmap / AMCL 全部超时，而 /scan 看起来完全正常。')
        print(f'{INFO} 检查：URDF 里有 {frame} 这个 link 和固定关节吗？'
              f'robot_state_publisher 起来了吗？')
        print(f'{INFO}   ros2 run tf2_ros tf2_echo {base_frame} {frame}')
        return

    t = tf.transform.translation
    q = tf.transform.rotation
    # 四元数 -> yaw（只关心绕 z 的安装偏角，车体上的雷达不会有横滚俯仰诉求）
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    print(f'{OK} {base_frame} -> {frame} 可解析')
    print(f'{INFO} 平移 xyz = ({t.x:+.4f}, {t.y:+.4f}, {t.z:+.4f}) m')
    print(f'{INFO} 偏角 yaw = {math.degrees(yaw):+.2f}°')
    print(f'{INFO} ★ 这三个数 + yaw 必须与**卷尺/实物**对得上：')
    print(f'{INFO}   x/y 是雷达光心相对车体中心的位置，z 是光心离地高度，')
    print(f'{INFO}   yaw 是安装偏角。差几厘米不致命，差几十厘米会让地图整体')
    print(f'{INFO}   旋转、定位持续偏移。量出来的值填进 URDF 的')
    print(f'{INFO}   lidar_mount_x/y/z（acg720_real.urdf.xacro:173）。')
    if abs(t.z) < 0.01:
        print(f'{WARN} 光心离地高度 z≈0 —— 要么还没量、要么雷达真在地面高度，'
              f'确认一下')


def check_rate(probe: ScanProbe, duration: float) -> None:
    section(f'4. 发布频率（收 {duration:.0f} s）')
    before = len(probe.msgs)
    probe.spin_for(duration)
    got = probe.msgs[before:]
    if not got:
        print(f'{BAD} 这 {duration:.0f} s 里一帧都没有 —— 刚才是偶发还是已经停了？')
        hard('扫描不连续（观测窗口内没有数据）')
        return

    # 用 header.stamp 算，避免 sim time / wall time 混用造成的假读数；
    # 时间戳不往前走时退化成"消息数 / 墙钟时间"。
    t0 = got[0].header.stamp
    t1 = got[-1].header.stamp
    stamp_span = (t1.sec - t0.sec) + (t1.nanosec - t0.nanosec) * 1e-9
    if len(got) > 1 and stamp_span > 0:
        rate = (len(got) - 1) / stamp_span
        print(f'{INFO} 收到 {len(got)} 帧，按 header.stamp 算是 {rate:.1f} Hz')
    else:
        rate = len(got) / duration
        print(f'{WARN} header.stamp 没有推进（全是同一个时刻，或 sim time 没开）')
        print(f'{INFO} 退回按墙钟算：{len(got)} 帧 / {duration:.0f} s = '
              f'{rate:.1f} Hz')

    if rate < MIN_RATE_HZ:
        print(f'{WARN} {rate:.1f} Hz 偏低（<{MIN_RATE_HZ:.0f} Hz）。'
              f'AMCL 与 costmap 会明显变迟钝，但还能跑')
    else:
        print(f'{OK} 频率够用')

    # sim time 与 wall time 的混用是"数据被判过期丢掉"的经典根因
    now = probe.get_clock().now()
    age = (now.nanoseconds - t1.sec * 10**9 - t1.nanosec) / 1e9
    if abs(age) > 1.0:
        print(f'{WARN} 最新一帧的时间戳与节点当前时间差 {age:+.1f} s')
        print(f'{INFO} 两边可能一个在用 sim time、一个在用 wall time。'
              f'检查 use_sim_time 是否一致（真机上应当是 false）')
    else:
        print(f'{OK} 时间戳新鲜（差 {age:+.3f} s）')


def check_quality(msg: LaserScan) -> None:
    section('5. 数据质量与盲区')
    st = analyse_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                      msg.range_min, msg.range_max)
    n = st['total']
    print(f'{INFO} 有效点 {st["valid"]}/{n}（{st["valid_ratio"] * 100:.1f}%）')
    # 三种"没有东西"的表示法分开报 —— 驱动用哪一种会决定下游怎么判，
    # 混成一个数字就看不出差别了。
    print(f'{INFO} 无回波：NaN/inf {st["nan_or_inf"]} 个，'
          f'恰等于 range_max {st["no_return"]} 个，'
          f'低于 range_min {st["below_min"]} 个')
    if st['min_valid'] is not None:
        print(f'{INFO} 实测距离范围 [{st["min_valid"]:.3f}, '
              f'{st["max_valid"]:.3f}] m')

    if st['valid'] == 0:
        print(f'{BAD} 一个有效点都没有 —— 数据不可用')
        hard('整帧扫描没有一个有效点')
        return

    if st['min_valid'] is not None and st['min_valid'] < 0.15:
        print(f'{WARN} 最近点只有 {st["min_valid"]:.3f} m。'
              f'真雷达在空旷处不该看到这么近的东西：')
        print(f'{INFO}   要么是自遮挡（车体挡在雷达前），要么是镜面/玻璃的假点，')
        print(f'{INFO}   要么雷达装得太低扫到了地面。用 costmap 之前先弄明白')

    if st['blind_sectors']:
        print(f'{WARN} 疑似盲区/遮挡扇区 {len(st["blind_sectors"])} 个：')
        for start, end, ratio in st['blind_sectors']:
            print(f'{INFO}   [{start:7.1f}°, {end:7.1f}°) 无回波 '
                  f'{ratio * 100:.0f}%')
        print(f'{INFO} 如果这些角度与车体自身重合 -> 是自遮挡。'
              f'不排除的话 costmap 会在这些方向永久留一个贴着的障碍。')
        print(f'{INFO} 处理办法：装得高一点/挪位置，或把该扇区的点裁掉后再喂给 Nav2。')
    else:
        print(f'{OK} 没有整块盲区（每 10° 扇区都有回波）')


def _run_checks(probe: ScanProbe, args) -> bool:
    """按固定顺序跑完五节检查。返回是否收到了数据。"""
    got = check_topic(probe, args.topic, args.timeout)
    if not got:
        return False
    check_metadata(probe.msgs[-1], args.expect_frame)
    check_tf(probe, probe.msgs[-1].header.frame_id, args.base_frame)
    check_rate(probe, args.duration)
    # 频率测完再取最新一帧做质量分析，避免用到刚起时的首帧
    check_quality(probe.msgs[-1])
    return True


def main() -> int:
    ap = argparse.ArgumentParser(
        description='2D 雷达（/scan）就绪自检，只读订阅，不发任何命令')
    ap.add_argument('--topic', default=EXPECT_TOPIC, help='扫描话题')
    ap.add_argument('--expect-frame', default=EXPECT_FRAME,
                    help='工程约定的雷达 frame 名')
    ap.add_argument('--base-frame', default=BASE_FRAME, help='车体 frame 名')
    ap.add_argument('--timeout', type=float, default=10.0,
                    help='等第一帧的超时（秒）')
    ap.add_argument('--duration', type=float, default=5.0,
                    help='测频率的观测窗口（秒）')
    ap.add_argument('-q', '--quiet', action='store_true',
                    help='只打印有问题（✗/⚠）的项')
    ap.add_argument('--selftest', action='store_true',
                    help='不连 ROS，用构造数据验证统计逻辑')
    args = ap.parse_args()

    if args.selftest:
        return _selftest()

    print('=' * 68)
    print('2D 雷达（/scan）就绪自检  ——  只读，不发任何命令')
    print('=' * 68)

    rclpy.init()
    probe = ScanProbe(args.topic)
    try:
        # -q 时把输出吞掉再选择性重放；默认**不缓冲**，因为整个检查要十几秒
        # （等首帧 + 测频率），缓冲住会让人以为脚本卡死了。
        if args.quiet:
            import contextlib
            import io
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                _run_checks(probe, args)
            for line in buf.getvalue().splitlines():
                if line.startswith((BAD, WARN, '===')):
                    print(line)
        else:
            _run_checks(probe, args)
    finally:
        probe.destroy_node()
        rclpy.shutdown()

    print('\n' + '=' * 68)
    if _blockers:
        print(f'有 {len(_blockers)} 项硬阻塞 —— 雷达这段现在还接不上：')
        for i, b in enumerate(_blockers, 1):
            print(f'  {i}. {b}')
        print('=' * 68)
        return 1
    print('雷达这段通了 ✓')
    print('下一步：建图（slam.launch.py / nav.launch.py slam:=true，推车走一圈）')
    print('        real_robot_params.md 第 2 节：标 wheel_separation_scale')
    print('=' * 68)
    return 0


if __name__ == '__main__':
    sys.exit(main())
