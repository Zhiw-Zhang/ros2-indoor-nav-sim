#!/usr/bin/env python3
"""端到端硬件在环（HIL）测试：**真驱动节点** + **真 PTY** + 虚拟 ACG720。

它与 `test_protocol_and_kinematics.py` 的分工：

    单测文件：钉住纯函数（编解码、运动学、积分），不需要 ROS，毫秒级
    本文件：  钉住**集成行为**——把真正的 Acg720Driver 节点接到真正的 PTY 上，
              看 /cmd_vel 进去之后 /odom、TF、位姿是不是对的

所以这里验证的是"接线对不对、符号对不对、尺度对不对"，这些正是真机上
最难现场排查的东西。

跑法（需要创建 PTY，所以要能访问 /dev/ptmx）::

    python3 src/my_robot_description/test/test_hil_end_to_end.py

退出码 0 = 全部通过。
"""

import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, "..", "src")
sys.path.insert(0, _SRC)
sys.path.insert(0, _HERE)

from acg720_driver import Acg720Driver            # noqa: E402
from acg720_simulator import Acg720SimulatorNode, PtySerial   # noqa: E402
from drive_kinematics import SkidSteerGeometry    # noqa: E402


class Harness:
    """把虚拟车 + 驱动节点 + 一个外部旋进器装在同一条 spin 上。

    为什么要"同进程 + 单线程 executor + spin_once"而不是分进程：
    分进程的话，"命令发出去到车动起来"的时序要靠 sleep 凑，测试会 flaky；
    这里每个 tick 物理推进 10 ms、ROS 回调也跑一轮，时序是确定的。
    """

    #: 虚拟车的真值有效轮距倍率。代表"真实车体"的侧滑程度。
    GT_SCALE = 1.36

    def __init__(self, driver_scale=1.0, publish_tf=True):
        self.pty = PtySerial()
        self.sim = Acg720SimulatorNode(
            self.pty,
            physics_rate_hz=100.0,      # 每 tick 10 ms，让控制/物理时序更细
            telemetry_rate_hz=20.0,     # 协议规定的 20 Hz 不能改
            ground_truth_separation_scale=self.GT_SCALE,
        )
        # ★ 必须在**建节点时**传参数：驱动的 __init__ 末尾就会打开串口并按
        #   参数算运动学，建完再 set_parameters 已经太晚（串口已按默认值打开）。
        self.driver = Acg720Driver(parameter_overrides={
            "port": self.pty.slave_path,
            "wheel_separation_scale": driver_scale,
            "publish_tf": publish_tf,
            # 放宽驱动侧限幅，让测试能单独考察 FPGA 侧的拒绝逻辑
            "max_linear_vel": 2.0,
            "max_angular_vel": 6.0,
            "max_wheel_rpm": 200.0,
            "min_wheel_rpm": -200.0,
        })

        self.cmd_pub = self.driver.create_publisher(Twist, "cmd_vel", 10)
        self.odoms = []
        self.driver.create_subscription(Odometry, "odom",
                                        lambda m: self.odoms.append(m), 20)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.sim)
        self.executor.add_node(self.driver)

    # ---------------- 驱动 ----------------

    def spin_for(self, seconds, cmd=None, rate=0.02, publish_cmd=True):
        """跑 seconds 秒。cmd=(v, ω) 时按 rate 持续发 /cmd_vel。"""
        end = time.monotonic() + seconds
        last_cmd = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if cmd is not None and publish_cmd and now - last_cmd >= rate:
                last_cmd = now
                msg = Twist()
                msg.linear.x = float(cmd[0])
                msg.angular.z = float(cmd[1])
                self.cmd_pub.publish(msg)
            self.executor.spin_once(timeout_sec=0.002)

    def latest_odom(self):
        return self.odoms[-1] if self.odoms else None

    def reset_pose(self):
        """把驱动侧积分位姿**和虚拟车真值位姿**一起清零。

        ⚠ 必须两边都清。只清驱动侧的话，下一步测试比的是"里程计增量 vs 累计真值"，
        比值会莫名其妙地偏小（本项目就踩过：单元测试里 ratio 变成 0.68 而不是 1.36）。
        这种污染在单独跑某一项时不会出现，最容易被误判成"物理模型不对"。
        """
        self.driver.pose.x = 0.0
        self.driver.pose.y = 0.0
        self.driver.pose.yaw = 0.0
        self.sim.model.x = 0.0
        self.sim.model.y = 0.0
        self.sim.model.yaw = 0.0
        self.odoms.clear()

    def rates_over(self, seconds, cmd, skip=1.0, rate=0.02):
        """同一时间窗内同时采样里程计与真值，返回 (w_odom, w_true)。

        ⚠ 两条信号必须**同窗**测量。分别测再相除是不对的：电机是一阶响应，
        两次测量的起步瞬态不同，比值会带上与尺度无关的误差。
        偏航角在这段时间里也可能跨过 ±π 回绕，所以用增量展开求和，
        而不是"末减初"。
        """
        t0 = time.monotonic()
        samples = []          # (t, yaw_odom, yaw_true)
        end = t0 + seconds
        last_cmd = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last_cmd >= rate:
                last_cmd = now
                self.cmd_pub.publish(_twist(*cmd))
            self.executor.spin_once(timeout_sec=0.002)
            o = self.latest_odom()
            if o is not None:
                samples.append((now - t0,
                                _yaw_from_quat(o.pose.pose.orientation),
                                self.sim.model.yaw))
        late = [s for s in samples if s[0] >= skip]
        if len(late) < 3:
            return float("nan"), float("nan")
        dt = late[-1][0] - late[0][0]
        if dt <= 0:
            return float("nan"), float("nan")
        return (_unwrap_total([s[1] for s in late]) / dt,
                _unwrap_total([s[2] for s in late]) / dt)

    def close(self):
        try:
            self.driver.destroy_node()
            self.sim.destroy_node()
        except Exception:
            pass
        self.pty.close()


# ==========================================================================
# 断言工具
# ==========================================================================

class Failures:
    def __init__(self):
        self.items = []

    def check(self, name, ok, detail=""):
        mark = "✓" if ok else "✗"
        print(f"  {mark} {name}" + (f"  {detail}" if detail else ""))
        if not ok:
            self.items.append(name)

    def approx(self, name, got, want, tol, unit=""):
        ok = abs(got - want) <= tol
        self.check(name, ok,
                   f"got={got:.4f}{unit} want={want:.4f}±{tol}{unit}")
        return ok


def nominal_rpm_for(v, geom):
    return geom.mps_to_rpm(v)


# ==========================================================================
# 各项测试
# ==========================================================================

def test_link_come_up(f: Failures, h: Harness):
    """① 串口打开、遥测开始流动、TF 链出现。"""
    print("\n[1] 链路起来")
    h.spin_for(1.5, cmd=(0.0, 0.0))
    f.check("串口已连接", h.driver.serial_connected)
    f.check("收到遥测帧", h.driver.telemetry_count > 10,
            f"telemetry={h.driver.telemetry_count}")
    f.check("CRC 无错误", h.driver._parser.crc_errors == 0,
            f"crc_errors={h.driver._parser.crc_errors}")
    f.check("有 /odom 发布", len(h.odoms) > 10, f"odom_msgs={len(h.odoms)}")
    o = h.latest_odom()
    if o is not None:
        f.check("odom frame 正确",
                o.header.frame_id == "odom" and o.child_frame_id == "base_link",
                f"{o.header.frame_id} -> {o.child_frame_id}")
    # 链路正常时不应该上报任何错误旗标
    f.check("无异常旗标（心跳未超时）",
            not (h.sim.model.flags & 0x01),
            f"flags=0x{h.sim.model.flags:04X}")


def test_straight_line(f: Failures, h: Harness):
    """② 直行：命令 0.15 m/s 走 2 s，里程计位移应接近 0.30 m。"""
    print("\n[2] 直行 0.15 m/s × 2 s")
    h.reset_pose()
    h.spin_for(2.0, cmd=(0.15, 0.0))
    o = h.latest_odom()
    if o is None:
        f.check("有 odom", False)
        return
    dx = o.pose.pose.position.x
    dy = o.pose.pose.position.y
    # 2 s 里前 ~0.15 s 是电机一阶启动，所以实际里程略小于 0.30
    f.check("位移在合理区间（0.20~0.31 m）", 0.20 <= dx <= 0.31,
            f"x={dx:.4f} m")
    f.check("没有横向漂移", abs(dy) < 0.01, f"y={dy:.5f} m")
    f.approx("航向未变", o.twist.twist.angular.z, 0.0, 0.05, " rad/s")
    # 左右轮目标必须相等（直行）
    a, b, c, d = h.driver._last_sent
    f.check("四轮目标一致（直行）",
            max(a, b, c, d) - min(a, b, c, d) <= 1,
            f"A/B/C/D=({a:.0f},{b:.0f},{c:.0f},{d:.0f})")


def test_rotation_direction(f: Failures, h: Harness):
    """③ 原地转方向：正 ω 必须是逆时针（右轮快），这是最易搞反的一项。"""
    print("\n[3] 原地旋转方向与符号")
    h.spin_for(0.3, cmd=(0.0, 0.0))
    h.cmd_pub.publish(_twist(0.0, 1.0))
    h.spin_for(0.3)
    a, b, c, d = h.driver._last_sent
    # 左旋 ω>0 → B、C（左轮）应为负，A、D（右轮）应为正
    f.check("左轮反转（B、C < 0）", b < 0 and c < 0,
            f"B={b:.1f} C={c:.1f}")
    f.check("右轮正转（A、D > 0）", a > 0 and d > 0,
            f"A={a:.1f} D={d:.1f}")
    f.check("右侧快于左侧", (a + d) > (b + c),
            f"R={ (a+d)/2:.1f} L={(b+c)/2:.1f}")

    h.reset_pose()
    h.spin_for(1.5, cmd=(0.0, 1.0))
    o = h.latest_odom()
    if o is not None:
        f.check("正 ω 产生正 yaw（逆时针）", o.twist.twist.angular.z > 0,
                f"w_odom={o.twist.twist.angular.z:.3f} rad/s")


def test_odom_scale_uncorrected(f: Failures, h: Harness):
    """④ 未标定（scale=1.0）时，里程计会**高报**转角约 1.36 倍。

    这就是真车第一次上手会看到的现象：命令转 1 圈，里程计报 1 圈，
    但车实际只转了 0.73 圈。本测试把这个偏差钉下来，作为标定前的基线。

    口径说明：用**稳态角速度之比**（θ_odom 的斜率 / θ_true 的斜率），
    而不是末态角度相除——后者带 ±π 回绕歧义和起步瞬态误差。
    """
    print("\n[4] 未标定基线：scale=1.0，里程计高报转角")
    h.reset_pose()
    w_odom, w_true = h.rates_over(3.0, (0.0, 0.8), skip=1.2)
    if not (abs(w_true) > 1e-3):
        f.check("真值有转动", False, f"w_true={w_true}")
        return
    ratio = w_odom / w_true
    f.check("里程计高报（ratio > 1.15）", ratio > 1.15,
            f"θ_odom/θ_true = {ratio:.3f}"
            f"（里程计 {w_odom:.3f} vs 真值 {w_true:.3f} rad/s）")
    f.check("高报倍率接近真值侧滑系数 1.36",
            abs(ratio - Harness.GT_SCALE) < 0.12,
            f"ratio={ratio:.3f} vs gt={Harness.GT_SCALE}")


def test_odom_scale_corrected(f: Failures):
    """⑤ 标定（scale=1.36）后，里程计转角应贴合真值（误差 <5%）。

    这是真机标定流程的验收口径：原地转固定时间，比 θ_odom 与 θ_true。
    本测试证明"把 scale 参数填对"确实能把这个误差收掉。
    """
    print("\n[5] 标定后：scale=1.36，里程计贴合真值")
    h = Harness(driver_scale=Harness.GT_SCALE)
    try:
        h.spin_for(1.5, cmd=(0.0, 0.0))
        h.reset_pose()
        w_odom, w_true = h.rates_over(3.0, (0.0, 0.8), skip=1.2)
        if not (abs(w_true) > 1e-3):
            f.check("真值有转动", False, f"w_true={w_true}")
            return
        ratio = w_odom / w_true
        f.check("θ_odom ≈ θ_true（误差 <5%）", abs(ratio - 1.0) < 0.05,
                f"θ_odom/θ_true = {ratio:.4f}"
                f"（里程计 {w_odom:.3f} vs 真值 {w_true:.3f} rad/s）")
    finally:
        h.close()


def test_cmd_timeout_stops(f: Failures, h: Harness):
    """⑥ 命令超时必须停车 —— 真机安全的核心一条，仿真里不存在。

    用 `ros2 param set` 等价的方式在线改超时，顺便验证"运行时改参数生效"
    （驱动刻意在用到时才读 cmd_vel_timeout，不缓存）。
    """
    print("\n[6] /cmd_vel 超时停车")
    h.driver.set_parameters([
        rclpy.parameter.Parameter("cmd_vel_timeout", value=0.3)])
    h.spin_for(0.6, cmd=(0.2, 0.0))
    sent_before = h.driver._last_sent
    f.check("超时前在给速度", max(abs(x) for x in sent_before) > 1,
            f"last_rpm={tuple(round(x,1) for x in sent_before)}")
    # 停止发指令，等超时
    h.spin_for(0.8, publish_cmd=False)
    sent_after = h.driver._last_sent
    f.check("超时后下发零速", max(abs(x) for x in sent_after) <= 1,
            f"last_rpm={tuple(round(x,1) for x in sent_after)}")
    # 车真的慢下来
    h.spin_for(0.4)
    rpm = h.sim.model.wheels[0].rpm
    f.check("车轮确实停了", abs(rpm) < 1.0, f"wheel_A={rpm:.3f} RPM")


def test_heartbeat_timeout_safe_state(f: Failures):
    """⑦ 不发心跳时 FPGA 侧必须进安全态（复刻协议 300 ms 超时）。

    这是上车前必查的一条安全链：上位机死了/卡了，底盘必须自己停。
    用 `send_heartbeat:=false` 模拟"上位机不再发心跳"。
    """
    print("\n[7] 心跳超时 → 虚拟车进安全态")
    h = Harness()
    try:
        h.driver.set_parameters([
            rclpy.parameter.Parameter("send_heartbeat", value=False)])
        h.spin_for(1.2, cmd=(0.2, 0.0))
        f.check("FPGA 进入心跳超时安全态",
                bool(h.sim.model.flags & 0x01),
                f"flags=0x{h.sim.model.flags:04X}")
        f.check("真值轮速被清零", abs(h.sim.model.wheels[0].rpm) < 1.0,
                f"wheel_A={h.sim.model.wheels[0].rpm:.3f} RPM")
    finally:
        h.close()


def test_low_speed_floor(f: Failures, h: Harness):
    """⑧ 低速目标复现真车缺陷：内轮目标低于 7 RPM 时车会滑行。

    这直接决定避障减速段能不能用，所以必须在无硬件时就看到。
    """
    print("\n[8] 低速跟踪下限（真车已知缺陷）")
    h.reset_pose()
    geom = SkidSteerGeometry(scale=1.0)
    # 选一个会让慢轮低于 7 RPM 的小角速度
    l, r = geom.twist_to_wheels(0.02, 0.10)
    f.check("构造出的慢轮目标确实低于下限", min(abs(l), abs(r)) < 7.0,
            f"L={l:.2f} R={r:.2f} RPM")
    h.spin_for(1.2, cmd=(0.02, 0.10))
    slow_idx = 1 if abs(l) < abs(r) else 0     # 慢的一侧（左轮组为 B,C）
    slow_rpm = h.sim.model.wheels[slow_idx].rpm
    f.check("慢轮实际速度被抬到 ~7 RPM（不是 0）", abs(slow_rpm) > 5.0,
            f"慢轮实际 = {abs(slow_rpm):.2f} RPM")
    f.check("驱动发出过低速告警",
            h.driver.low_speed_warn_count > 0 or
            bool(h.sim.model.flags & 0x40),
            f"driver_warn={h.driver.low_speed_warn_count} "
            f"flags=0x{h.sim.model.flags:04X}")


def test_target_rejection(f: Failures, h: Harness):
    """⑨ 超过 FPGA 上限的目标必须被拒，且驱动要打 WARN（不能静默）。

    ⚠ 构造要点：驱动侧限幅必须**低于** FPGA 的 200 RPM 上限，否则驱动会先把
    目标剪到 200 以内，FPGA 就永远看不到"超限"这个请求，测试也就永远不会
    触发拒绝路径——本项目第一次写这个测试时正是这么写错的（挂了 500 RPM），
    结果是"测了但什么都没测到"。
    """
    print("\n[9] FPGA 拒绝 >200 RPM 目标")
    # 驱动限幅放到 400：目标 441 会被剪到 400，仍然 >200，
    # 于是这个"超限请求"真的送到了 FPGA，拒绝路径才被测到。
    # （驱动侧限幅在用时才读参数，所以 set_parameters 能立刻生效。）
    h.driver.set_parameters([
        rclpy.parameter.Parameter("max_wheel_rpm", value=400.0),
        rclpy.parameter.Parameter("min_wheel_rpm", value=-400.0),
        rclpy.parameter.Parameter("max_linear_vel", value=5.0)])
    before = h.driver.rejected_count
    h.spin_for(1.2, cmd=(1.5, 0.0))          # 1.5 m/s ≈ 441 RPM ≫ 200
    f.check("驱动收到过拒绝回报", h.driver.rejected_count > before,
            f"rejected {before} → {h.driver.rejected_count}")
    f.check("虚拟车记下 TARGET_REJECT 旗标",
            bool(h.sim.model.flags & 0x10),
            f"flags=0x{h.sim.model.flags:04X}")
    # 被拒的目标**不能被执行**：FPGA 拒绝时应保留上一条有效目标，
    # 而不是把 400 RPM 真发到电机上
    f.check("被拒目标没有被执行（轮速未达 400 RPM）",
            max(abs(w.rpm) for w in h.sim.model.wheels) < 300.0,
            f"wheels={[round(w.rpm,2) for w in h.sim.model.wheels]}")


def test_disconnect_recovery(f: Failures):
    """⑩ 串口"断开"后必须自动重连，而不是躺着不动。"""
    print("\n[10] 串口断开自动重连")
    h = Harness()
    try:
        h.spin_for(1.0, cmd=(0.0, 0.0))
        f.check("初始已连接", h.driver.serial_connected)
        # 模拟掉线：关掉驱动的串口
        h.driver._close_serial()
        f.check("已标记断开", not h.driver.serial_connected)
        h.driver.set_parameters([
            rclpy.parameter.Parameter("reconnect_period", value=0.3)])
        # 注意：PTY slave 路径一直存在，所以重连应当成功
        h.spin_for(1.5)
        f.check("已自动重连", h.driver.serial_connected)
    finally:
        h.close()


def _twist(v, w):
    m = Twist()
    m.linear.x = float(v)
    m.angular.z = float(w)
    return m


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _unwrap_total(yaws):
    """把一串回绕在 ±π 的角度展开后，返回**累计变化量**。

    直接用 `y[-1] - y[0]` 在跨过 ±π 时会得到 2π 量级的错误跳变，
    所以逐点把差值折进 (-π, π] 再累加。
    """
    total = 0.0
    for a, b in zip(yaws, yaws[1:]):
        d = b - a
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        total += d
    return total


# ==========================================================================

def main():
    rclpy.init()
    f = Failures()
    results = []
    print("=" * 72)
    print("ACG720 硬件在环（HIL）端到端测试：真驱动节点 + 真 PTY + 虚拟车")
    print("=" * 72)

    # 前面几项共用一个 harness（快）；涉及不同 scale/超时的各起一个
    h = Harness(driver_scale=1.0)
    try:
        for name, fn in [
            ("链路起来", test_link_come_up),
            ("直行", test_straight_line),
            ("旋转方向", test_rotation_direction),
            ("未标定里程计", test_odom_scale_uncorrected),
        ]:
            try:
                fn(f, h)
            except Exception as exc:
                f.check(f"{name} 抛异常", False, repr(exc))
    finally:
        h.close()

    for name, fn in [
        ("标定后里程计", test_odom_scale_corrected),
        ("心跳超时安全态", test_heartbeat_timeout_safe_state),
        ("串口重连", test_disconnect_recovery),
    ]:
        try:
            fn(f)
        except Exception as exc:
            f.check(f"{name} 抛异常", False, repr(exc))

    h2 = Harness(driver_scale=1.0)
    try:
        for name, fn in [("命令超时停车", test_cmd_timeout_stops),
                         ("低速下限", test_low_speed_floor),
                         ("目标拒绝", test_target_rejection)]:
            try:
                fn(f, h2)
            except Exception as exc:
                f.check(f"{name} 抛异常", False, repr(exc))
    finally:
        h2.close()

    print("\n" + "=" * 72)
    if f.items:
        print(f"失败 {len(f.items)} 项：")
        for i in f.items:
            print(f"  - {i}")
        print("=" * 72)
        return 1
    print("全部通过 ✓")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
