#!/usr/bin/env python3
"""ACG720 底盘驱动节点：ROS 2 ↔ FPGA 串口。

它是仿真里那个 `gz-sim-diff-drive-system` 插件的**替代品**，职责完全对应：

    gz 插件（仿真）                    本节点（真机）
    ─────────────────────────────      ─────────────────────────────
    <topic>cmd_vel</topic>        →    /cmd_vel  订阅
    差分运动学积分                →    drive_kinematics.SkidSteerGeometry
    发布 /odom + odom→base_link   →    同样，但积分源是**编码器计数**
    内部真值位姿                  →    没有了（真机不存在，这是迁移最大的坑）

设计上刻意做的几个选择
----------------------
1. **位姿由编码器计数积分，不用 FPGA 上报的 RPM 字段。**
   交接包说得很清楚：1320 CPR 和 RPM 换算公式都还【待确认】。RPM 是 FPGA 用这个
   待确认公式算出来的二手值，一旦公式错，用它积分的位姿会**持续漂移**（误差随时间
   累积）；而计数是一手的，尺度错了也只是整体比例错，可以靠标定一次性修好。
   所以：计数积分 → 位姿；RPM 只用于**诊断显示**（对比两者能立刻看出 CPR 对不对）。

2. **cmd_vel 超时必停。**
   仿真里车不会因为"上位机崩了"而自己乱跑。真机必须。超时（默认 0.5 s）没收到
   新指令就下发零速。这比 FPGA 自己的 300 ms 心跳超时更靠前一道。

3. **失败要吵。**
   串口打不开、CRC 一直错、目标被 FPGA 拒绝——这些必须报 ERROR/WARN，
   否则症状是"launch 起来了、话题也在、车就是不动"，现场很难查。

4. **不假装能做的事。**
   FPGA 拒绝 > 200 RPM 的目标、内轮低于约 7 RPM 跟不上（交接包 F1 验收实测）。
   节点会把这些如实转成**限幅 + 告警**，而不是默默发一个车执行不了的目标。
"""

from __future__ import annotations

import math
import os
import queue
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy)
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

import serial
from serial import SerialException

# 让脚本既能 `ros2 run` 也能直接 python3 执行
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from acg720_protocol import (  # noqa: E402
    Cmd, Flag, FrameParser, REJECT_NAMES, Telemetry,
    encode, pack_heartbeat, pack_velocity, parse_status, parse_telemetry,
    wheels_to_lr, lr_to_wheels,
)
from drive_kinematics import (  # noqa: E402
    Pose2D, SkidSteerGeometry, quaternion_from_yaw, wrap_angle,
)


#: 一帧遥测里带的诊断信息，发到 /acg720/diagnostics（std_msgs/String，人可读）。
#: 用 String 而不是自定义 msg，是为了不引入 msg 生成——现场用 `ros2 topic echo`
#: 就能直接看懂，也不需要重新 build。
class Acg720Driver(Node):
    """按协议驱动 ACG720 底盘，并把它表现成一个标准的 ROS 2 移动底盘。"""

    def __init__(self, parameter_overrides: Optional[dict] = None):
        """parameter_overrides 用来在**建节点时**就定好参数。

        ⚠ 为什么不能"先建节点再 set_parameters"：本节点的 __init__ 末尾就会
        打开串口并按参数算运动学，之后再改 port 已经晚了——串口已经按旧值
        打开（甚至已经失败）。测试和 launch 都需要在建节点前把值定下来。
        值写成 {"port": "/dev/pts/3", ...} 这种 dict，内部会转成 ROS 参数类型。
        """
        super().__init__("acg720_driver",
                         parameter_overrides=self._to_ros_params(parameter_overrides))

        # ---------------- 参数 ----------------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        # 几何 / 标定（全部可在 launch 里覆盖，真机标定后只改这些值）
        self.declare_parameter("wheel_radius_m", 0.0325)      # 【待确认】65 mm 轮径
        self.declare_parameter("wheel_separation_m", 0.200)   # 【待确认】200 mm 轮距
        self.declare_parameter("wheelbase_m", 0.185)          # 【待确认】185 mm 轴距
        self.declare_parameter("counts_per_rev", 1320.0)      # 【待确认】修正 2
        #: ★ skid-steer 有效轮距系数。真机初始 1.0，必须按实测标定，不要抄仿真的 1.36
        self.declare_parameter("wheel_separation_scale", 1.0)

        # 速度限幅。默认取仿真里那套偏保守的值，真机标定前不要放大。
        self.declare_parameter("max_linear_vel", 0.20)        # m/s
        self.declare_parameter("max_angular_vel", 1.0)        # rad/s
        self.declare_parameter("max_wheel_rpm", 40.0)         # 协议取值范围 8–40
        self.declare_parameter("min_wheel_rpm", -40.0)
        #: 协议下界：交接包给的目标范围是 8–40 RPM，低于约 7 RPM 跟不上
        self.declare_parameter("rpm_below_tracking_floor", 7.0)

        # 时序
        self.declare_parameter("cmd_vel_timeout", 0.5)        # s，超时停车
        self.declare_parameter("heartbeat_period", 0.1)       # s，协议要求 100 ms
        #: 关掉心跳发送。**只在台架上验证 FPGA 安全态时用**，正常跑必须为 true
        self.declare_parameter("send_heartbeat", True)
        self.declare_parameter("status_period", 1.0)          # s，诊断信息周期
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("reconnect_period", 1.0)       # s，串口掉线重连
        self.declare_parameter("dry_run", False)              # 只算不发，用于干跑

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.dry_run = self.get_parameter("dry_run").value

        self.geom = SkidSteerGeometry(
            wheel_radius_m=self.get_parameter("wheel_radius_m").value,
            wheel_separation_m=self.get_parameter("wheel_separation_m").value,
            wheelbase_m=self.get_parameter("wheelbase_m").value,
            counts_per_rev=self.get_parameter("counts_per_rev").value,
            scale=self.get_parameter("wheel_separation_scale").value,
        )
        # 速度限幅等参数在 _send_twist 里**每次读**，不在这里缓存。
        # 缓存过一次就变成"ros2 param set 改了没反应"，真机现场调试会白折腾半天
        # （本项目就踩过：测试里把 max_wheel_rpm 从 200 改到 400，
        #   节点仍然按旧的 200 限幅，于是"超限请求"根本没送到 FPGA，
        #   拒绝路径的测试静默失效）。
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        # 注意：cmd_vel_timeout / heartbeat_period / reconnect_period / status_period
        # 刻意**不**在这里缓存，而是在每次用到时 get_parameter() 读。
        # 理由：真机现场经常需要在线改这几个时序（比如调试时想放宽超时），
        # `ros2 param set` 只有在用到时才读才生效；缓存了就成了"改了没反应"。

        # ---------------- 状态 ----------------
        self.pose = Pose2D()
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.last_cmd_time: Optional[float] = None
        self.last_telemetry_time: Optional[float] = None
        self.heartbeat_seq = 0
        self.last_uptime_ms: Optional[int] = None
        self.telemetry_count = 0
        self.rejected_count = 0
        self.clamped_count = 0
        self.low_speed_warn_count = 0
        self._last_telemetry: Optional[Telemetry] = None
        self._active_flags = 0
        self._cmd_count = 0
        self.serial_connected = False

        # 串口线程 → ROS 线程 的单向队列。串口读不能被回调阻塞，
        # 所以读线程只负责"收字节 + 解析 + 入队"，全部逻辑留在定时器里。
        self._frame_queue: "queue.Queue" = queue.Queue(maxsize=1000)
        self._parser = FrameParser()
        self._ser: Optional[serial.Serial] = None
        self._ser_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None

        # ---------------- ROS 接口 ----------------
        # 订阅 /cmd_vel：与仿真里 gz 插件的 <topic>cmd_vel</topic> 同名，
        # 所以 nav2_params.yaml 里几十处引用一个字都不用改。
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

        # odom 用 reliable + transient 没必要，用默认的 sensor-data 风格即可
        self.odom_pub = self.create_publisher(Odometry, "odom", 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        #: 原始遥测（给标定脚本用：能看到 FPGA 自己算的 RPM，用来反推 CPR）
        self.raw_pub = self.create_publisher(String, "acg720/telemetry_raw", 10)
        self.diag_pub = self.create_publisher(String, "acg720/diagnostics", 10)
        self.diag_latched = self.create_publisher(
            String, "acg720/diagnostics_latched",
            QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        hb_period = self.get_parameter("heartbeat_period").value
        self.create_timer(hb_period, self._on_heartbeat)
        self.create_timer(0.02, self._pump_serial)      # 50 Hz，与遥测周期一致
        self.create_timer(self.get_parameter("status_period").value, self._on_status_timer)

        self._latch(
            f"acg720_driver starting: port={self.port} baud={self.baudrate} "
            f"scale={self.geom.scale} b_eff={self.geom.effective_separation_m:.4f} m "
            f"rpm_per_mps={self.geom.rpm_per_mps:.1f} "
            f"max_v={self.get_parameter('max_linear_vel').value} "
            f"max_w={self.get_parameter('max_angular_vel').value} "
            f"max_rpm={self.get_parameter('max_wheel_rpm').value}"
        )
        self._open_serial()

    # ==================================================================
    # 参数辅助
    # ==================================================================

    #: 参数名 → ROS 参数类型。用来把普通 dict 转成 Parameter 列表，
    #: 免得上层调用方去 import rclpy.parameter。类型必须和 declare_parameter
    #: 的默认值一致，否则 rclpy 会因类型不匹配抛异常。
    _PARAM_TYPES = {
        "port": str,
        "baudrate": int,
        "wheel_radius_m": float,
        "wheel_separation_m": float,
        "wheelbase_m": float,
        "counts_per_rev": float,
        "wheel_separation_scale": float,
        "max_linear_vel": float,
        "max_angular_vel": float,
        "max_wheel_rpm": float,
        "min_wheel_rpm": float,
        "rpm_below_tracking_floor": float,
        "cmd_vel_timeout": float,
        "heartbeat_period": float,
        "send_heartbeat": bool,
        "status_period": float,
        "odom_frame": str,
        "base_frame": str,
        "publish_tf": bool,
        "reconnect_period": float,
        "dry_run": bool,
    }

    @classmethod
    def _to_ros_params(cls, overrides: Optional[dict]):
        if not overrides:
            return None
        from rclpy.parameter import Parameter
        params = []
        for key, value in overrides.items():
            if key not in cls._PARAM_TYPES:
                raise KeyError(
                    f"未知参数 {key!r}；可用：{sorted(cls._PARAM_TYPES)}")
            params.append(Parameter(key, value=value))
        return params

    # ==================================================================
    # 串口
    # ==================================================================

    def _open_serial(self) -> None:
        if self.dry_run:
            self.get_logger().warn("dry_run=true：不打开串口，只做运动学计算")
            self.serial_connected = False
            return
        try:
            with self._ser_lock:
                self._ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.05,
                    write_timeout=0.5,
                    # 真机接 USB 转串口时，这两项不关掉可能一上电就被拉低复位
                    rtscts=False,
                    dsrdtr=False,
                )
            self.serial_connected = True
            self._stop.clear()
            self._reader_thread = threading.Thread(
                target=self._read_loop, name="acg720-reader", daemon=True)
            self._reader_thread.start()
            self.get_logger().info(f"串口已打开：{self.port} @ {self.baudrate}")
            self._latch(f"serial OPEN {self.port} @ {self.baudrate}")
        except (SerialException, OSError) as exc:
            self.serial_connected = False
            self.get_logger().error(
                f"串口打开失败：{self.port} → {exc}。"
                f"将每 {self.get_parameter('reconnect_period').value}s 重试。"
                f"（检查：设备是否存在、dialout 组权限、是否被别的进程占用）"
            )
            self._latch(f"serial OPEN FAILED {self.port}: {exc}")

    def _close_serial(self) -> None:
        self._stop.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None
        with self._ser_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
        if self.serial_connected:
            self.get_logger().warn("串口已关闭")
        self.serial_connected = False

    def _read_loop(self) -> None:
        """后台线程：只做"读字节 → 解析 → 入队"，不做任何 ROS 调用。"""
        while not self._stop.is_set():
            ser = self._ser
            if ser is None:
                time.sleep(0.05)
                continue
            try:
                data = ser.read(256)
            except (SerialException, OSError) as exc:
                self._frame_queue.put(("error", str(exc)))
                time.sleep(0.05)
                continue
            if not data:
                continue
            for frame in self._parser.feed(data):
                try:
                    self._frame_queue.put_nowait(("frame", frame))
                except queue.Full:
                    pass   # 队列满说明消费端卡住，丢帧好过阻塞串口

    def _write(self, blob: bytes) -> bool:
        if self.dry_run or not self.serial_connected:
            return False
        try:
            with self._ser_lock:
                if self._ser is None:
                    return False
                self._ser.write(blob)
            return True
        except (SerialException, OSError) as exc:
            self.get_logger().error(f"串口写失败：{exc}")
            self.serial_connected = False
            return False

    # ==================================================================
    # 定时器回调
    # ==================================================================

    def _pump_serial(self) -> None:
        """消费解析线程送来的帧，并处理断线重连。"""
        if not self.dry_run and not self.serial_connected:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - getattr(self, "_last_reconnect", 0.0) >= \
                    self.get_parameter("reconnect_period").value:
                self._last_reconnect = now
                self._open_serial()

        handled = 0
        while handled < 50:                     # 每周期最多处理 50 帧，避免长时间占住回调
            try:
                kind, item = self._frame_queue.get_nowait()
            except queue.Empty:
                break
            handled += 1
            if kind == "error":
                self.get_logger().error(f"串口读错误：{item}（准备重连）")
                self._close_serial()
                continue
            self._handle_frame(item)

        # 安全：指令超时 → 停车。这是真机与仿真最关键的一处不同。
        if self.last_cmd_time is not None:
            now = self.get_clock().now().nanoseconds / 1e9
            cmd_timeout = self.get_parameter("cmd_vel_timeout").value
            if now - self.last_cmd_time > cmd_timeout:
                self.last_cmd_time = None
                self.last_cmd_v = 0.0
                self.last_cmd_w = 0.0
                self.get_logger().warn(
                    f"{cmd_timeout}s 没收到 /cmd_vel，下发零速停车",
                    throttle_duration_sec=5.0)
                self._send_twist(0.0, 0.0)

    def _handle_frame(self, frame) -> None:
        if frame.cmd == Cmd.TELEMETRY:
            try:
                t = parse_telemetry(frame.payload)
            except ValueError as exc:
                self.get_logger().error(f"遥测解析失败：{exc}")
                return
            self._on_telemetry(t)
        elif frame.cmd == Cmd.STATUS:
            try:
                s = parse_status(frame.payload)
            except ValueError as exc:
                self.get_logger().error(f"状态解析失败：{exc}")
                return
            if not s.accepted:
                self.rejected_count += 1
                name = REJECT_NAMES.get(s.reject_code, f"code={s.reject_code}")
                self.get_logger().warn(
                    f"FPGA 拒绝了上一条指令：{name}", throttle_duration_sec=2.0)
            self._active_flags = 0     # 状态帧不带 flags，保持简单
        elif frame.cmd == Cmd.POSITION_DONE:
            self.get_logger().info(f"位置命令完成回报：{frame.payload.hex(' ')}")
        else:
            self.get_logger().warn(
                f"收到未定义命令字 0x{frame.cmd:02X}，payload={frame.payload.hex(' ')}",
                throttle_duration_sec=5.0)

    def _on_telemetry(self, t: Telemetry) -> None:
        """收到一帧遥测：积分位姿、发 odom、发 TF、发诊断。"""
        now_ns = self.get_clock().now().nanoseconds
        self.telemetry_count += 1

        # --- 时间戳：优先用 FPGA 的 uptime 差，退路是用本机时钟差 ---
        # 用本机时钟而不是"固定 50 ms"，是因为遥测实际周期会抖；用固定值会让
        # 速度估计带上周期性误差，AMCL 会看到速度跳变。
        if self.last_telemetry_time is None:
            dt = 0.05
        else:
            dt = max((now_ns - self.last_telemetry_time) / 1e9, 1e-4)
        self.last_telemetry_time = now_ns
        self.last_uptime_ms = t.uptime_ms
        self._active_flags = t.flags
        self._last_telemetry = t

        # --- 一手数据：编码器计数 → 左右轮位移 → 位姿 ---
        left_counts = t.counts[1] + t.counts[2]      # B + C
        right_counts = t.counts[0] + t.counts[3]     # A + D
        dl = self.geom.counts_to_distance_m(left_counts / 2.0)
        dr = self.geom.counts_to_distance_m(right_counts / 2.0)
        d_center = (dl + dr) / 2.0
        d_yaw = (dr - dl) / self.geom.effective_separation_m
        self.pose = Pose2D(
            x=self.pose.x + d_center * math.cos(self.pose.yaw + d_yaw / 2.0),
            y=self.pose.y + d_center * math.sin(self.pose.yaw + d_yaw / 2.0),
            yaw=wrap_angle(self.pose.yaw + d_yaw),
        )

        v = d_center / dt
        omega = d_yaw / dt

        # --- 发布 /odom ---
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.pose.x
        odom.pose.pose.position.y = self.pose.y
        qx, qy, qz, qw = quaternion_from_yaw(self.pose.yaw)
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        # 协方差：真机上这项有意义（仿真里是插件拍的值）。先用保守估计，
        # 等实测出重复定位精度再改小，否则 AMCL 会过度信任里程计。
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.02
        self.odom_pub.publish(odom)

        if self.get_parameter("publish_tf").value:
            tf = TransformStamped()
            tf.header.stamp = odom.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.pose.x
            tf.transform.translation.y = self.pose.y
            tf.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            self.tf_broadcaster.sendTransform(tf)

        # --- 诊断：比对 FPGA 上报的 RPM 与我们自己算的，用来反查 CPR ---
        fpga_left, fpga_right = wheels_to_lr(*t.rpm)
        msg = String()
        msg.data = (
            f"n={self.telemetry_count} seq={t.seq} dt={dt*1000:.1f}ms "
            f"counts=({t.counts[0]},{t.counts[1]},{t.counts[2]},{t.counts[3]}) "
            f"rpm=({t.rpm[0]},{t.rpm[1]},{t.rpm[2]},{t.rpm[3]}) "
            f"duty=({t.duty[0]},{t.duty[1]},{t.duty[2]},{t.duty[3]}) "
            f"v={v:.4f} w={omega:.4f} flags=0x{t.flags:04X}"
        )
        self.raw_pub.publish(msg)

        if t.flags & Flag.LOW_SPEED_WARN:
            self.low_speed_warn_count += 1
        if t.flags & (Flag.ESTOP | Flag.STALL):
            names = ",".join(t.flag_names())
            self.get_logger().error(f"FPGA 报告异常状态：{names}",
                                    throttle_duration_sec=1.0)

    def _send_twist(self, v: float, omega: float) -> None:
        """把 (v, ω) 变成四轮 RPM 目标并下发。"""
        # 全部在用时才读参数：现场 `ros2 param set` 可以立刻生效，无需重启
        max_v = self.get_parameter("max_linear_vel").value
        max_w = self.get_parameter("max_angular_vel").value
        max_rpm = self.get_parameter("max_wheel_rpm").value
        min_rpm = self.get_parameter("min_wheel_rpm").value
        track_floor = self.get_parameter("rpm_below_tracking_floor").value

        v = max(-max_v, min(max_v, v))
        omega = max(-max_w, min(max_w, omega))

        left_rpm, right_rpm = self.geom.twist_to_wheels(v, omega)

        # 限幅。这里**必须**告警而不是静默剪切：FPGA 会拒绝 >200 RPM 的目标，
        # 但真正的坑是**内轮低于约 7 RPM 时车会以 7 RPM 滑行**（交接包 F1 实测），
        # 表现为"让它慢速微调，它却冲出去"。
        clamped = left_rpm > max_rpm or left_rpm < min_rpm or \
            right_rpm > max_rpm or right_rpm < min_rpm
        left_rpm = max(min_rpm, min(max_rpm, left_rpm))
        right_rpm = max(min_rpm, min(max_rpm, right_rpm))
        if clamped:
            self.clamped_count += 1
            self.get_logger().warn(
                f"轮速目标被限幅到 [{min_rpm}, {max_rpm}] RPM"
                f"（L={left_rpm:.1f} R={right_rpm:.1f}）",
                throttle_duration_sec=2.0)

        # 低速跟踪下限提醒：只提醒，不改指令——要不要抬速度是策略问题，
        # 由上层决定（车的物理行为改不了）。
        mags = [abs(x) for x in (left_rpm, right_rpm) if abs(x) > 1e-6]
        if mags and min(mags) < track_floor:
            self.get_logger().warn(
                f"内轮目标 {min(mags):.1f} RPM 低于跟踪下限 {track_floor} RPM，"
                f"车实际会以约 {track_floor} RPM 滑行，转向会比预期小",
                throttle_duration_sec=3.0)

        a, b, c, d = lr_to_wheels(left_rpm, right_rpm)
        self._write(encode(Cmd.SET_VELOCITY, pack_velocity(a, b, c, d)))
        self._cmd_count += 1
        self._last_sent = (a, b, c, d)

    # ==================================================================
    # 订阅 / 心跳
    # ==================================================================

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.last_cmd_time = self.get_clock().now().nanoseconds / 1e9
        self.last_cmd_v = msg.linear.x
        self.last_cmd_w = msg.angular.z
        self._send_twist(self.last_cmd_v, self.last_cmd_w)

    def _on_heartbeat(self) -> None:
        """协议要求 100 ms 一次；FPGA 侧 300 ms 超时会进安全态。

        每次读 send_heartbeat 参数而不是缓存，这样可以用
        `ros2 param set /acg720_driver send_heartbeat false` 在线停掉心跳——
        专门用来**验证 FPGA 的安全态真的会触发**（这是上车前必须确认的一条）。
        """
        if not self.get_parameter("send_heartbeat").value:
            return
        self.heartbeat_seq = (self.heartbeat_seq + 1) & 0xFF
        self._write(encode(Cmd.HEARTBEAT, pack_heartbeat(self.heartbeat_seq)))

    def _on_status_timer(self) -> None:
        now = time.time()
        parts = [
            f"connected={self.serial_connected}",
            f"telemetry={self.telemetry_count}",
            f"cmds={self._cmd_count}",
            f"rejected={self.rejected_count}",
            f"clamped={self.clamped_count}",
            f"low_speed_warn={self.low_speed_warn_count}",
            f"crc_errors={self._parser.crc_errors}",
            f"dropped_bytes={self._parser.dropped_bytes}",
            f"unknown_cmds={self._parser.unknown_cmd[-5:]}",
            f"pose=({self.pose.x:.3f},{self.pose.y:.3f},{math.degrees(self.pose.yaw):.1f}deg)",
        ]
        if self.last_telemetry_time is not None:
            age = self.get_clock().now().nanoseconds / 1e9 - self.last_telemetry_time
            parts.append(f"telemetry_age={age:.2f}s")
            if age > 0.5:
                self.get_logger().error(
                    f"{age:.2f}s 没有收到遥测帧——检查串口/波特率/接线")
        if getattr(self, "_last_sent", None):
            parts.append("last_rpm=({:.1f},{:.1f},{:.1f},{:.1f})".format(*self._last_sent))
        self.diag_pub.publish(String(data=" ".join(parts)))

    # ==================================================================

    def _latch(self, text: str) -> None:
        """发一条 latched 诊断。latched = 后连上来的订阅者也能看到，
        用来回答"这个节点上次启动到底成没成功"。"""
        msg = String(data=text)
        self.diag_latched.publish(msg)

    def destroy_node(self):
        # 退出前先停车，否则下一次启动时电机会保持最后的指令
        try:
            self._send_twist(0.0, 0.0)
            time.sleep(0.05)
        except Exception:
            pass
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Acg720Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
