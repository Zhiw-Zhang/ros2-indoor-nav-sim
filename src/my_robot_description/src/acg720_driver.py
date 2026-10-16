#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ACG720 底盘驱动节点：ROS 2 ↔ FPGA 串口（真协议 V1）。

它是仿真里那个 `gz-sim-diff-drive-system` 插件的**替代品**，职责完全对应：

    gz 插件（仿真）                    本节点（真机）
    ─────────────────────────────────  ─────────────────────────────
    <topic>cmd_vel</topic>        →    /cmd_vel  订阅
    差分运动学积分                →    drive_kinematics.SkidSteerGeometry
    发布 /odom + odom→base_link   →    同样，但积分源是**编码器计数**
    内部真值位姿                  →    没有了（真机不存在，这是迁移最大的坑）

⚠️ 本文件是**重写**，不是改参数
------------------------------
上一版说的是本项目自己发明的协议（没有 VER/SEQ/LEN、CRC 用 MODBUS）。
真固件在第 3 个字节就会判版本错并**永久锁死底盘**。所以那一版不是"参数不准"，
而是"根本说不上一句话"。现在改用 `acg720_protocol_v1.py`（与厂商实现逐字节
交叉核对过）。

设计上刻意做的几个选择
----------------------
1. **位姿由编码器累计计数积分**（0x83 的 `position_count_abcd`，s32 会回绕，
   用 `delta32` 取差量）。不直接用上报 RPM：计数是一手的，尺度错了也只是整体
   比例错，可以靠标定一次性修好；二手 RPM 的公式一旦错，误差会随时间累积。

2. **喂狗方式随状态而变，这是必须做对的一件事。**
   固件要求"300 ms 内至少收到一条有效命令"，否则进 FAULT_TIMEOUT。
   但 `top.v:238-281` 里**任何非 MOVE 的有效命令都会把运动目标清零** ——
   所以运动期间发 HEARTBEAT 会把车**刹停**（状态机却还停在 RUNNING）。
   于是：**运动期间靠重发 MOVE 喂狗，静止保持 ARM 时才发 HEARTBEAT。**

3. **命令要限速。** ACK 在固件里只有**单槽**，若它排在一条在途周期遥测后面
   （20 Hz 那批四包共 164 字节 ≈ 14 ms），此时再来一条被接受的命令就会置
   `tx_overflow`，而 `top.v:231` 把它 OR 进了 `protocol_fault` → **永久锁死**。
   所以有 `min_command_interval`（默认 50 ms = 20 Hz）。

4. **启动就校验位流版本号。** 0x83 帧里的 `build_id` 与厂商 `position_control.py`
   的做法一致：不符就**拒绝任何运动命令**。烧错位流时我们要的是"一眼看出"，
   不是"车按错误的速度跑而日志正常"。

5. **绝不重复发 ARM。** 固件里 ARM 只从 DISARMED 转 ARMED_STOP，重复发没有意义。

6. **不做"周期 STOP 绕过 3 秒上限"。** 那条上限是固件刻意的安全策略，
   交接包明令禁止绕过（`INTERFACE.md`）。连续运动超过 3.0 s 就会锁存
   `FAULT_RUN_LIMIT`，本节点把这件事**如实报出来**并停止发运动命令 ——
   这属于"该改 FPGA"，不是"该在主机上想办法"。

7. **失败要吵。** 串口打不开、CRC 一直错、底盘锁死、位流不对——都必须报
   ERROR/WARN，否则症状是"launch 起来了、话题也在、车就是不动"。

8. **`control_mode="position"`：用 0x05 位置段做连续导航，绕开 3.0 s 上限。**
   位置环**不受** `RUN_LIMIT` 约束（`chassis_safety_fsm.v:57-58` 里的
   `!position_active`），所以"一串位置段"可以在**不改 FPGA** 的前提下连续走。
   代价是**没有圆弧** —— 位置环只有前进/后退/原地左旋/原地右旋四种动作
   （`chassis_position_controller.v:44-47`），轨迹只能是折线。

   分段策略本身在 `position_segments.py`（纯逻辑、有单测，不依赖 ROS），
   本节点只负责接线：收目标 → 发 POSITION → 用 0x84 判段结束 → 发下一段。

   position 模式下有三条硬规矩，违反任何一条都会出真问题：

   * 段进行中**只发 HEARTBEAT**，绝不发 MOVE/ARM/DISARM/OPEN_LOOP ——
     `top.v:121-125` 把这四种 kind 当成 `position_abort`，会把当前段**中断**。
   * 段结束那一拍 `busy` 落回 0 之后，`target_rpm` 立刻切回**手动目标**
     （`top.v:152` 的三目运算符）。所以 position 模式**绝不能同时下 MOVE**，
     否则段一结束，车会带着残留目标继续冲出去。
   * 段下发前**自己再校验一遍参数**：非法的 POSITION 会被判
     `protocol_fault` → **永久锁死底盘**，不是"报个错"。见 `_maybe_send_position`。
"""

from __future__ import annotations

import math
import os
import queue
import sys
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

import serial
from serial import SerialException

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from acg720_firmware import fault_cn  # noqa: E402
from acg720_protocol_v1 import (  # noqa: E402
    FaultReason, FrameParser, Kind, ProtocolError, SafetyState, SequenceTracker,
    delta32, encode_command, pack_move, pack_position, parse_ack,
    parse_diagnostic, parse_position_telemetry, parse_telemetry,
)
from drive_kinematics import (  # noqa: E402
    MoveCommand, Pose2D, SkidSteerGeometry, quaternion_from_yaw, twist_to_move,
    wrap_angle, yaw_from_quaternion,
)
from position_segments import (  # noqa: E402
    ActionKind, ExecState, Segment, SegmentExecutor, SegmentParams,
    SegmentPlanner, TargetPoint,
)

#: 底盘安全状态的中文名（与 acg720_firmware 的故障表配套）
STATE_CN = {
    SafetyState.DISARMED: "DISARMED（未使能）",
    SafetyState.ARMED_STOP: "ARMED_STOP（已使能、停止）",
    SafetyState.RUNNING: "RUNNING（正在运动）",
    SafetyState.FAULT: "FAULT（锁死）",
}


class Acg720Driver(Node):
    """按真协议 V1 驱动 ACG720 底盘，并把它表现成一个标准的 ROS 2 移动底盘。"""

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
        self.declare_parameter("wheel_radius_m", 0.0325)      # 65 mm 轮径
        self.declare_parameter("wheel_separation_m", 0.200)   # 200 mm 轮距
        self.declare_parameter("wheelbase_m", 0.185)          # 185 mm 轴距
        self.declare_parameter("counts_per_rev", 1320.0)      # encoder_quadrature.v:6
        #: ★ skid-steer 有效轮距系数。真机初始 1.0，必须按实测标定
        self.declare_parameter("wheel_separation_scale", 1.0)

        # 速度限幅
        self.declare_parameter("max_linear_vel", 0.20)        # m/s
        self.declare_parameter("max_angular_vel", 1.0)        # rad/s
        #: 固件对 MOVE 的 base_rpm 上限（chassis_command_rx.v:99）
        self.declare_parameter("max_base_rpm", 200.0)
        #: RTL 硬下限：内侧轮不能反转 → 最小转弯半径 = 有效轮距/2
        self.declare_parameter("min_turn_radius_factor", 0.5)
        #: 仅**告警**：低于此转速车会以滑行下限跑（交接包 F1 实测≈7 RPM）
        self.declare_parameter("tracking_floor_rpm", 7.0)

        # 协议 / 安全
        #: "move"     = 主机把 (v, ω) 反解成 0x02 MOVE（连续运动，**受 3 s 上限**）
        #: "position" = 主机把目标点拆成 0x05 POSITION 段（折线，**不受 3 s 上限**）
        self.declare_parameter("control_mode", "move")
        #: 位流版本号（chassis_telemetry_imu_tx.v:40）。空串 = 不校验
        self.declare_parameter("expected_build_id", "20260A02")
        self.declare_parameter("auto_arm", True)
        #: 运动期间重发 MOVE 的周期。**不是心跳**（见模块 docstring 第 2 条）
        self.declare_parameter("command_period", 0.1)
        #: 命令最小间隔，防 ACK 单槽撞车（见模块 docstring 第 3 条）
        self.declare_parameter("min_command_interval", 0.05)
        #: 在真实串口上使用连续运动前必须显式确认：3.0 s 后会锁死（固件限制）
        self.declare_parameter("allow_run_limit", False)

        # ---- position 模式专用 ----
        #: 目标点话题（geometry_msgs/PoseStamped）。可以 remap 成 /goal_pose，
        #: 这样 RViz 的 "2D Goal Pose" 工具就能直接派目标。
        self.declare_parameter("goal_topic", "acg720/goal")
        #: 目标里是否使用朝向。默认 False —— RViz/多数工具发的是"到点即可"，
        #: 朝向字段往往是垃圾值，当成约束会把车逼着原地乱转。
        self.declare_parameter("goal_uses_yaw", False)
        #: 非零 /cmd_vel 视为"人工接管"，立刻中止当前目标。默认开：
        #: 摇杆永远该能压过自主导航，这是台架调试时的保命开关。
        self.declare_parameter("cmd_vel_overrides_goal", True)
        #: 分段策略参数，透传给 position_segments.SegmentParams
        self.declare_parameter("segment_heading_tolerance_rad", 0.10)
        self.declare_parameter("segment_position_tolerance_m", 0.03)
        self.declare_parameter("segment_straight_rpm", 30)
        self.declare_parameter("segment_turn_rpm", 20)
        self.declare_parameter("segment_max_bad_streak", 3)
        self.declare_parameter("segment_max_without_progress", 6)

        # 时序
        self.declare_parameter("cmd_vel_timeout", 0.5)        # s，超时停车
        self.declare_parameter("status_period", 1.0)          # s，诊断信息周期
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        #: s，串口掉线重连。★ 必须**明显小于**固件的 300 ms 命令超时：
        #:   断开超过 300 ms，固件会锁存 FAULT_TIMEOUT，之后重连成功也没用，
        #:   只能断电/按 S4 复位。HIL 测试把这条失败模式钉住了。
        self.declare_parameter("reconnect_period", 0.1)
        self.declare_parameter("dry_run", False)              # 只算不发

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.dry_run = self.get_parameter("dry_run").value
        self.control_mode = self.get_parameter("control_mode").value
        if self.control_mode not in ("move", "position"):
            raise ValueError(
                f"control_mode={self.control_mode!r} 不支持。只有两种："
                f"'move'（(v, ω) → 0x02 MOVE，连续但**受 3 s 上限**）和 "
                f"'position'（目标点 → 一串 0x05 POSITION，折线但不受上限）。")

        self.geom = SkidSteerGeometry(
            wheel_radius_m=self.get_parameter("wheel_radius_m").value,
            wheel_separation_m=self.get_parameter("wheel_separation_m").value,
            wheelbase_m=self.get_parameter("wheelbase_m").value,
            counts_per_rev=self.get_parameter("counts_per_rev").value,
            scale=self.get_parameter("wheel_separation_scale").value,
        )
        # 限幅等参数在用到时**每次读**，不缓存：缓存过一次就变成
        # "ros2 param set 改了没反应"，真机现场调试会白折腾半天。
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # ---------------- 状态 ----------------
        self.pose = Pose2D()
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.last_cmd_time: Optional[float] = None
        self.cmd_timed_out = False

        self.seq = SequenceTracker()
        self.seq_synced = False
        self.arm_sent = False
        self.motion_blocked = False
        self.block_reason = ""
        self.build_id: Optional[int] = None
        self.build_id_ok: Optional[bool] = None

        self.fw_state: Optional[SafetyState] = None
        self.fw_fault = FaultReason.NONE
        self.fw_run_age_blocked = False
        self.position_busy = False
        self.position_done = False
        self.position_result: Optional[int] = None
        self.position_progress = 0
        #: 上一次看到的 0x84 `done` **电平**，用来做上升沿检测。
        #: `done` 在固件里是电平（段完成置 1、下一段 start 才清 0），
        #: 而 0x84 是 20 Hz 周期广播 —— 不去重就会把上一段的完成帧
        #: 当成新段的结果。详见 `_handle_frame`。
        self._position_done_level = False

        # ---- position 模式：目标点 → 位置段 ----
        # 分段策略是纯逻辑（position_segments.py），这里只接线。
        self.segment_params: Optional[SegmentParams] = None
        self.segment_planner: Optional[SegmentPlanner] = None
        self.segment_executor: Optional[SegmentExecutor] = None
        self.segment_ack_count = 0
        #: 已下发但还没被 0x84 确认结束的那一段（只用于日志）
        self._inflight_segment: Optional[Segment] = None
        #: 0x84 报来的 (result_code, progress)，等下一拍在发送路径上消费。
        #: ★ 不在这里直接触发"发下一段"：所有串口写都必须走同一个地方，
        #:   否则限速/SEQ 的记账会有两个说法。
        self._pending_segment_result: Optional[Tuple[int, int]] = None
        self._need_stop = False
        self._goal_seq = 0

        # ★ 位置模式的规划器/执行器必须在这里构造 —— **在状态初始化之后**。
        #   曾经把它放在上面的几何参数旁边，结果被这里的 `= None` 初始化覆盖掉，
        #   症状是 `_on_goal` 直接 return（目标像石沉大海）、`_position_tick`
        #   断言炸掉。构造依赖 `self.geom` 和参数，所以位置也只能在这之后。
        if self.control_mode == "position":
            # 分段参数一次性读出来。刻意不支持运行中改：改到一半会让"上一段用旧
            # rpm、下一段用新 rpm"，而段长上限是按 rpm 反算的，混着用可能撞上
            # 固件的段内超时（11.95 s）。
            self.segment_params = SegmentParams(
                heading_tolerance_rad=self.get_parameter(
                    "segment_heading_tolerance_rad").value,
                position_tolerance_m=self.get_parameter(
                    "segment_position_tolerance_m").value,
                straight_rpm=int(self.get_parameter("segment_straight_rpm").value),
                turn_rpm=int(self.get_parameter("segment_turn_rpm").value),
                max_bad_streak=int(self.get_parameter(
                    "segment_max_bad_streak").value),
                max_segments_without_progress=int(self.get_parameter(
                    "segment_max_without_progress").value),
            )
            # 构造失败（容差配得根本没法指令等）就在这里抛出来 —— 这是好事：
            # 宁可起不来，也不要"起来了但每一段都发不出去"。
            self.segment_planner = SegmentPlanner(self.geom, self.segment_params)
            self.segment_executor = SegmentExecutor(self.segment_planner,
                                                    self.segment_params)

        self._prev_counts: Optional[List[int]] = None
        self.last_telemetry_time: Optional[float] = None
        self.telemetry_count = 0
        self.ack_count = 0
        self.cmd_count = 0
        self.motion_cmd_count = 0
        self.blocked_cmd_count = 0
        self.projection_count = 0
        self.rejected_twist_count = 0
        self.last_move: Optional[MoveCommand] = None
        self._last_tx_sig: Optional[Tuple] = None
        self._last_tx_kind: Optional[str] = None
        self._last_tx_time = 0.0
        self._last_fault_logged: Optional[FaultReason] = None
        self._fw_counters = {'crc': 0, 'dup': 0, 'frame': 0}
        self._open_count = 0
        self._last_reconnect = 0.0
        self._needs_resync = True
        self.serial_connected = False

        # 串口线程 → ROS 线程 的单向队列
        self._frame_queue: "queue.Queue" = queue.Queue(maxsize=1000)
        self._parser = FrameParser()
        self._ser: Optional[serial.Serial] = None
        self._ser_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None

        # ---------------- ROS 接口 ----------------
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

        self.odom_pub = self.create_publisher(Odometry, "odom", 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.raw_pub = self.create_publisher(String, "acg720/telemetry_raw", 10)
        self.diag_pub = self.create_publisher(String, "acg720/diagnostics", 10)
        self.diag_latched = self.create_publisher(
            String, "acg720/diagnostics_latched",
            QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        if self.control_mode == "position":
            # 目标点。用 PoseStamped 是为了能 remap 到 /goal_pose ——
            # 那样 RViz 的 "2D Goal Pose" 工具就直接能派目标。
            # ⚠ QoS 必须用**默认的**（RELIABLE + VOLATILE）。这里曾经写成
            #   TRANSIENT_LOCAL 想"后连上来的也能拿到目标"，那是错的：
            #   DDS 的 durability 匹配要求 offered ≥ requested，而 RViz 和
            #   `ros2 topic pub` 发的都是 VOLATILE —— 于是**根本连不上**，
            #   症状是"话题在、发了没反应、连个警告都没有"。
            #   latched 语义只对**状态上报**有意义（见下面的 segment_pub）。
            self.create_subscription(
                PoseStamped, self.get_parameter("goal_topic").value,
                self._on_goal, 1)
            # 目标可能来自 map 系（RViz / Nav2），所以需要一个 TF 监听来做变换。
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            # 段状态上报。latched = 后连上来的订阅者立刻知道当前到哪了。
            self.segment_pub = self.create_publisher(
                String, "acg720/segment_status", QoSProfile(
                    depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.create_timer(0.01, self._on_control_tick)      # 100 Hz 决策
        self.create_timer(self.get_parameter("status_period").value, self._on_status_timer)

        self._latch(
            f"acg720_driver starting: port={self.port} baud={self.baudrate} "
            f"scale={self.geom.scale} b_eff={self.geom.effective_separation_m:.4f} m "
            f"build_id={self.get_parameter('expected_build_id').value or '(不校验)'} "
            f"max_v={self.get_parameter('max_linear_vel').value} "
            f"max_w={self.get_parameter('max_angular_vel').value}"
        )
        self.get_logger().warn(
            "★ 当前固件的**连续运动上限是 3.0 s**（RUN_LIMIT，超时会锁存 FAULT 且"
            "只能复位解除）。这是固件限制，不是本节点的问题 —— 连续导航需要 FPGA 变更，"
            "详见 docs/fpga_change_request.md")

        self._check_run_limit_policy()
        self._open_serial()

    # ==================================================================
    # 参数辅助
    # ==================================================================

    #: 参数名 → ROS 参数类型
    _PARAM_TYPES = {
        "port": str, "baudrate": int,
        "wheel_radius_m": float, "wheel_separation_m": float, "wheelbase_m": float,
        "counts_per_rev": float, "wheel_separation_scale": float,
        "max_linear_vel": float, "max_angular_vel": float, "max_base_rpm": float,
        "min_turn_radius_factor": float, "tracking_floor_rpm": float,
        "control_mode": str, "expected_build_id": str, "auto_arm": bool,
        "command_period": float, "min_command_interval": float,
        "allow_run_limit": bool,
        "goal_topic": str, "goal_uses_yaw": bool,
        "cmd_vel_overrides_goal": bool,
        "segment_heading_tolerance_rad": float,
        "segment_position_tolerance_m": float,
        "segment_straight_rpm": int, "segment_turn_rpm": int,
        "segment_max_bad_streak": int, "segment_max_without_progress": int,
        "cmd_vel_timeout": float, "status_period": float,
        "odom_frame": str, "base_frame": str, "publish_tf": bool,
        "reconnect_period": float, "dry_run": bool,
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

    def _check_run_limit_policy(self) -> None:
        """在真实串口上用**连续运动**之前，要求显式确认过 3 s 上限。

        PTY（虚拟底盘）不算真实端口 —— 测试要能直接跑。

        ⚠ position 模式**不受这条限制**，也不走这个检查：它一条 MOVE 都不发，
          3 s 上限在固件里对位置环是关掉的（`chassis_safety_fsm.v:57-58` 的
          `!position_active`）。在这里拦住它等于把唯一的绕行方案也堵死。
        """
        if self.control_mode != "move":
            return
        if self.dry_run or "pts" in str(self.port):
            return
        if self.get_parameter("allow_run_limit").value:
            return
        raise RuntimeError(
            f"port={self.port} 看起来是真实串口，而连续运动会在 3.0 s 后把底盘"
            f"锁死在 FAULT_RUN_LIMIT（固件限制，只能断电/复位解除）。"
            f"确认你是在台架上做验证后，再加参数 allow_run_limit:=true。"
            f"（若想避开这条限制，用 control_mode:=position —— 它靠位置段走折线，"
            f"不触发 3 s 上限。）")

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
                    port=self.port, baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.05, write_timeout=0.5,
                    # 真机接 USB 转串口时，这两项不关掉可能一上电就被拉低复位
                    rtscts=False, dsrdtr=False,
                )
            self.serial_connected = True
            self._stop.clear()
            self._reader_thread = threading.Thread(
                target=self._read_loop, name="acg720-reader", daemon=True)
            self._reader_thread.start()
            with self._ser_lock:
                # ★ 丢掉打开之前积压在缓冲区里的字节。上一次连接（或上一次
                #   驱动实例）留下的**旧遥测**会让 SEQ 对齐到一个过期的
                #   last_command_seq，于是新发的命令被判成重复而静默丢弃。
                try:
                    self._ser.reset_input_buffer()
                except Exception:
                    pass
            self._open_count += 1
            self._last_reconnect = time.monotonic()
            self.get_logger().info(f"串口已打开：{self.port} @ {self.baudrate}")
            self._latch(f"serial OPEN {self.port} @ {self.baudrate}")
            # 重连后必须重来一遍：SEQ 要重新对齐、位流要重新校验、ARM 要重发
            self.seq_synced = False
            self._needs_resync = True
            self.arm_sent = False
            self.build_id_ok = None
            self._prev_counts = None
            self._parser.reset()
            self._last_tx_sig = None
            self._last_tx_time = 0.0
            self._last_tx_kind = None
        except (SerialException, OSError) as exc:
            self.serial_connected = False
            self.get_logger().error(
                f"串口打开失败：{self.port} → {exc}。"
                f"将每 {self.get_parameter('reconnect_period').value}s 重试。"
                f"（检查：设备是否存在、dialout 组权限、是否被别的进程占用）",
                throttle_duration_sec=5.0)
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
    # 主决策：100 Hz
    # ==================================================================

    def _on_control_tick(self) -> None:
        now = time.monotonic()

        # ---- 1) 断线重连 ----
        if not self.dry_run and not self.serial_connected:
            if now - getattr(self, "_last_reconnect", 0.0) >= \
                    self.get_parameter("reconnect_period").value:
                self._last_reconnect = now
                self._open_serial()

        # ---- 2) 消费收到的帧 ----
        handled = 0
        while handled < 50:
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

        # ---- 3) /cmd_vel 超时 → 目标清零（真机与仿真最关键的一处不同）----
        if self.last_cmd_time is not None:
            timeout = self.get_parameter("cmd_vel_timeout").value
            if now - self.last_cmd_time > timeout:
                self.last_cmd_time = None
                self.last_cmd_v = 0.0
                self.last_cmd_w = 0.0
                if not self.cmd_timed_out:
                    self.cmd_timed_out = True
                    self.get_logger().warn(
                        f"{timeout}s 没收到 /cmd_vel，下发停止",
                        throttle_duration_sec=5.0)

        # ---- 4) 该发什么 ----
        self._send_due_command(now)

    # ==================================================================
    # 下行：ARM / MOVE / STOP / HEARTBEAT
    # ==================================================================

    def _desired_twist(self) -> Tuple[float, float]:
        max_v = self.get_parameter("max_linear_vel").value
        max_w = self.get_parameter("max_angular_vel").value
        v = max(-max_v, min(max_v, self.last_cmd_v))
        w = max(-max_w, min(max_w, self.last_cmd_w))
        if self.last_cmd_time is None:
            return 0.0, 0.0
        return v, w

    def _send_due_command(self, now: float) -> None:
        if self.dry_run or not self.serial_connected or self.motion_blocked:
            return

        # ★ 序号对齐之前**一条都别发**。固件对"序号不新"的处理是**静默丢弃**
        #   （只加 duplicate 计数，不回 ACK、不报错），所以一条用错序号的帧
        #   既没用、又会在固件的 duplicate 计数上留下噪声，掩盖真问题。
        if not self.seq_synced:
            return
        # 位流没校验通过之前，一条运动命令都不发
        if self.build_id_ok is False:
            return

        # ---- 0) 有人要求停车（人工接管 / 目标被中止）----
        # 走的仍然是正常的限速路径，**不会**插到在途的 ACK 前面去。
        if self._need_stop:
            if self._send(Kind.STOP, b"", "stop"):
                self._need_stop = False
            return

        period = self.get_parameter("command_period").value

        # 还没 ARM：先 ARM（只发一次）。ARM 之前只发心跳，让固件知道主机活着
        if not self.arm_sent:
            if not self.get_parameter("auto_arm").value:
                return
            if self.build_id_ok is None:
                # 还没收到过 0x83；发心跳维持链路，等位流版本号
                self._send_heartbeat()
                return
            # ★ 只有**真的写出去了**才算 ARM 过。这里曾经写成
            #   `self._send(...); self.arm_sent = True`，而 `_send` 会因为
            #   `min_command_interval` 未到而静默不发 —— 于是 arm_sent 被置位、
            #   ARM 永远没发出去，症状是"状态机一直 DISARMED、车就是不动"。
            if self._send(Kind.ARM, b"", "arm"):
                self.arm_sent = True
                self.get_logger().info("已发送 ARM（使能底盘）")
            return

        # ---- position 模式：完全不看 /cmd_vel，只走位置段 ----
        # ★ 必须**在这里分叉**而不是在下面加个 if：move 模式的分支会发 MOVE，
        #   而 position 模式下一旦发出 MOVE，段一结束车就会带着残留手动目标
        #   继续冲（top.v:152）。两条路径必须互斥。
        if self.control_mode == "position":
            self._position_tick(now)
            return

        v, w = self._desired_twist()
        if abs(v) < 1e-9 and abs(w) < 1e-9:
            # 目标为零：RUNNING 时用 STOP 退出，其余情况用心跳保活
            if self.fw_state == SafetyState.RUNNING:
                self._send(Kind.STOP, b"", "stop")
                self._last_tx_kind = "heartbeat"      # 下一步转心跳
            else:
                self._send_heartbeat()
            return

        # ★ 运动期间**必须重发 MOVE**，不能发 HEARTBEAT：
        #   top.v 里任何非 MOVE 的有效命令都会把运动目标清零（车会刹停）
        mc = self._twist_to_move(v, w)
        if mc is None:
            self._send_heartbeat()
            return
        sig = (mc.motion_mode, mc.base_rpm, mc.turn_inner_ratio_q8)
        changed = sig != self._last_tx_sig
        due = (now - self._last_tx_time) >= period
        if not (changed or due) or not self._tx_due():
            return
        seq = self.seq.take()
        if not self._write(encode_command(Kind.MOVE, seq,
                                          pack_move(mc.motion_mode, mc.base_rpm,
                                                    mc.turn_inner_ratio_q8))):
            self.seq.sequence = seq
            return
        self._note_tx("move", sig)
        self.cmd_count += 1
        self.motion_cmd_count += 1
        self.last_move = mc

    # ==================================================================
    # position 模式：目标点 → 位置段
    # ==================================================================

    def _position_tick(self, now: float) -> None:
        """position 模式下每次该发命令时走这里。

        两条铁律（见模块 docstring 第 8 条）：
          * 段进行中只发 HEARTBEAT —— MOVE/ARM/DISARM/OPEN_LOOP 会 abort 当前段；
          * 绝不在这里发 MOVE —— 段结束时残留的手动目标会立刻接管。
        """
        ex = self.segment_executor
        assert ex is not None

        # 1) 先消费 0x84 报来的段结果（由 _handle_frame 存下，不在这里直接发）
        if self._pending_segment_result is not None:
            code, progress = self._pending_segment_result
            self._pending_segment_result = None
            self._inflight_segment = None
            action = ex.on_result(self.pose, code, progress, now)
            self._report_action(action, "段结果")
            # 本拍只做这一件事：要么已经把下一段发出去了，要么报了终态。
            # 不再往下走 —— 否则同一拍里还要再判一次"要不要发心跳"，纯属绕圈。
            return

        # 2) 没目标 / 在等段完成 → 只喂狗
        if not ex.active:
            self._send_heartbeat()
            return

        # 3) 限速没到就**本拍什么都不做**。
        #    不能"先规划再发"，因为规划会把执行器推到 WAITING，而发不出去时
        #    要回滚 —— 回滚本身是对的（见 on_send_deferred），但那样每 10 ms
        #    就白规划一次，日志和计数全是噪声。直接不进场最干净。
        if not self._tx_due():
            return

        action = ex.poll(self.pose, now)
        if action.kind is ActionKind.IDLE:
            # 正在等当前段走完。心跳既维持 300 ms 狗，又不会 abort 位置段
            # （top.v:121-125 里 position_abort 只认 0/2/3/4 四种 kind）。
            self._send_heartbeat()
            return
        self._report_action(action, "规划")
        if action.kind is ActionKind.SEND:
            return
        # ARRIVED / FAILED 已在 _report_action 里发布过状态

    def _maybe_send_position(self, seg: Segment) -> str:
        """下发一段 POSITION。返回 'sent' / 'deferred' / 'error'。

        ★ 这里有一道独立的参数校验，不是多余的：越界的 POSITION 会被固件判
          `protocol_fault`，而 `top.v:231` 把它 OR 进了安全状态机的锁存 ——
          后果是**永久锁死底盘**，不是"报个错重来"。规划器侧已有穷举单测
          （test_position_segments.py）保证不产出非法段，这里是第二道闸。
        """
        try:
            payload = pack_position(**seg.position_kwargs())
        except (ProtocolError, TypeError, ValueError) as exc:
            self.get_logger().error(
                f"★ 算出了一个非法的位置段，已拒绝下发（它会让固件 protocol_fault "
                f"并永久锁死底盘）：{seg.describe()} —— {exc}")
            self._latch(f"segment REJECTED_LOCALLY {seg.describe()} ({exc})")
            return "error"
        if not self._send(Kind.POSITION, payload, "position"):
            # `_send` 返回 False 有两种原因，必须分开：
            #   · 被 `min_command_interval` 挡掉 → **正常**，下拍再来；
            #   · 真的写失败了（串口没了）→ 要记账。
            # 区分办法：`_send` 只有在**通过**限速检查之后才可能写失败，
            # 而写失败不会更新 `_last_tx_time`。所以"限速仍然没到"= 前者。
            if self._tx_due():
                self.get_logger().error(f"位置段写入串口失败：{seg.describe()}")
                return "error"
            return "deferred"
        self._inflight_segment = seg
        ex = self.segment_executor
        if ex is not None:
            ex.note_sent()
        self.segment_ack_count += 1
        self.get_logger().info(
            f"位置段已下发：{seg.describe()}"
            + (f"（{seg.note}）" if seg.note else ""))
        return "sent"

    def _report_action(self, action, where: str) -> None:
        """把执行器的决策反映到日志/状态话题上，并在 SEND 时真正下发。"""
        ex = self.segment_executor
        assert ex is not None
        if action.kind is ActionKind.SEND:
            outcome = self._maybe_send_position(action.segment)
            if outcome == "error":
                # 参数非法 / 串口没了：要吵，而且要记账
                ex.on_send_failed("段参数非法或串口写失败")
            elif outcome == "deferred":
                # ★ 限速挡掉是**正常**的（每 5 拍就会发生一次）：退回去，
                #   下一拍再发，**不计失败**。这里曾经把它算进 bad_streak，
                #   三次就放弃 —— 症状是"目标刚下发就报失败、车一动不动"，
                #   而且取决于心跳与段的相位关系，看起来是随机的。
                ex.on_send_deferred()
            return
        if action.kind is ActionKind.ARRIVED:
            self.get_logger().info(f"★ 已到达目标点（{action.reason}）")
            self._publish_segment_status("ARRIVED", action.reason)
        elif action.kind is ActionKind.FAILED:
            self.get_logger().error(
                f"★ 放弃目标点（{where}）：{action.reason}。"
                f"车会停在当前位置，不会继续尝试。")
            self._latch(f"segment FAILED {action.reason}")
            self._publish_segment_status("FAILED", action.reason)

    def _publish_segment_status(self, phase: str, detail: str = "") -> None:
        ex = self.segment_executor
        if ex is None or not hasattr(self, "segment_pub"):
            return
        msg = String()
        msg.data = f"{phase} goal_seq={self._goal_seq} {ex.describe()}"
        if detail:
            msg.data += f" detail={detail}"
        self.segment_pub.publish(msg)

    def _on_goal(self, msg: PoseStamped) -> None:
        ex = self.segment_executor
        if ex is None:
            return
        if self.motion_blocked:
            self.get_logger().error(
                f"收到目标点但底盘已锁死（{self.block_reason}），拒绝执行")
            self._publish_segment_status("REJECTED", self.block_reason)
            return
        # 位流不符 = 我们连"这台车是什么行为"都不知道，绝不能动。
        # ★ 必须在这里**明确拒绝**，而不是"接受下来但什么都不发" ——
        #   后者会让调用方以为目标在跑，然后一直等一个永远不会来的 ARRIVED。
        if self.build_id_ok is False:
            reason = (f"位流版本不符（板上 0x{self.build_id:08X}，期望 "
                      f"{self.get_parameter('expected_build_id').value}），拒绝执行目标")
            self.get_logger().error(f"★ {reason}")
            self._publish_segment_status("REJECTED", reason)
            return

        # 目标可能在 map 系（RViz / Nav2 都发 map 系）。有 TF 就变换，没 TF 就只在
        # 本来就是 odom 系时接受 —— 绝不"当它是 odom 直接用"，那会让车往错的方向跑。
        frame = (msg.header.frame_id or "").strip()
        pose_in = msg
        if frame and frame != self.odom_frame:
            # 去掉前导斜杠再比较：'/odom' 与 'odom' 是同一个系
            if frame.lstrip("/") != self.odom_frame.lstrip("/"):
                try:
                    pose_in = self.tf_buffer.transform(
                        msg, self.odom_frame, timeout=Duration(seconds=0.3))
                except Exception as exc:                  # noqa: BLE001
                    self.get_logger().error(
                        f"目标点的坐标系是 {frame!r}，但拿不到它到 "
                        f"{self.odom_frame!r} 的 TF，无法执行：{exc}")
                    self._publish_segment_status("REJECTED", f"TF {frame}→odom 不可用")
                    return

        target = TargetPoint(
            x=float(pose_in.pose.position.x),
            y=float(pose_in.pose.position.y),
            yaw=self._goal_yaw(pose_in),
        )
        self._goal_seq += 1
        # 先停一下再开始：如果上一段还在跑，必须让固件把当前段 abort 掉，
        # 否则新目标的第一段会被 `start_valid && !busy` 静默丢弃（:177）。
        if ex.busy:
            self._need_stop = True
        ex.set_goal(target)
        self.get_logger().info(
            f"收到目标点 #{self._goal_seq}：({target.x:.3f}, {target.y:.3f})"
            + (f" yaw={math.degrees(target.yaw):.1f}°" if target.yaw is not None else ""))
        self._publish_segment_status("ACCEPTED")

    def _goal_yaw(self, msg: PoseStamped):
        """目标朝向。默认**不采用** —— 见 `goal_uses_yaw` 的说明。"""
        if not self.get_parameter("goal_uses_yaw").value:
            return None
        q = msg.pose.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return None                     # 全零四元数 = "没填"，不是"朝向 0"
        return yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _abort_goal(self, reason: str) -> None:
        """中止当前目标并停车（人工接管 / 参数越界 / 目标被替换）。"""
        ex = self.segment_executor
        if ex is None or not ex.active:
            return
        ex.cancel(reason)
        self._inflight_segment = None
        self._pending_segment_result = None
        self._need_stop = True              # STOP 会把在跑的位置段 abort 掉
        self.get_logger().warn(
            f"★ 已中止当前目标：{reason}。"
            f"（位置段是靠 STOP 中止的，见 top.v:121-125）")
        self._publish_segment_status("ABORTED", reason)

    def _send_heartbeat(self) -> None:
        """静止/保持 ARM 时的喂狗。

        ⚠ 运动期间**不能**用这个（`top.v:238-281` 会把运动目标清零），
          那时要重发 MOVE，见 `_send_due_command`。
        """
        period = self.get_parameter("command_period").value
        if (time.monotonic() - self._last_tx_time) < period \
                and self._last_tx_kind == "heartbeat":
            return
        self._send(Kind.HEARTBEAT, b"", "heartbeat")

    def _send(self, kind, payload: bytes, tag: str) -> bool:
        """发一条无载荷命令。返回**是否真的写出去了**。

        ⚠ 返回值不是装饰：调用方要靠它判断"ARM 到底发没发出去"。
          被 `min_command_interval` 挡掉是完全正常的（命令限速），
          但把"被挡掉"当成"已发送"会让状态机永远停在 DISARMED。

        ⚠⚠ **限速的判据在函数内部取时钟，不接受调用方传进来的时间。**
          这条是踩出来的：退出时曾经为了让 STOP/DISARM 立刻发出去，
          传了一个 `time.monotonic() + 10.0` 进去绕过限速 ——
          两条被接受的命令挤进同一个 ACK 槽 → 固件置 `tx_overflow`
          并**永久锁死**（`chassis_telemetry_imu_tx.v:236-238` + `top.v:231`）。
          也就是说，"主机为了让车停得更利索"反而把车搞锁死了。
          现在限速无法被绕过，这类自伤从结构上就不可能发生。
        """
        if not self._tx_due():
            return False
        seq = self.seq.take()
        if not self._write(encode_command(kind, seq, payload)):
            self.seq.sequence = seq          # 写失败就把 SEQ 退回去，别在固件侧留跳号
            return False
        self._note_tx(tag, None)
        self.cmd_count += 1
        return True

    def _tx_due(self) -> bool:
        return (time.monotonic() - self._last_tx_time) >= \
            self.get_parameter("min_command_interval").value

    def _note_tx(self, tag: str, sig) -> None:
        self._last_tx_time = time.monotonic()
        self._last_tx_kind = tag
        self._last_tx_sig = sig if tag == "move" else None

    def _twist_to_move(self, v: float, w: float) -> Optional[MoveCommand]:
        """(v, ω) → MOVE 参数，并把"投影"如实告警。"""
        max_base_rpm = int(self.get_parameter("max_base_rpm").value)
        try:
            mc = twist_to_move(v, w, self.geom, max_base_rpm)
        except ValueError as exc:
            self.get_logger().error(f"(v, ω) 反解失败：{exc}")
            self.rejected_twist_count += 1
            return None

        if not mc.achievable:
            self.projection_count += 1
            self.get_logger().warn(
                f"(v={v:.3f}, ω={w:.3f}) 固件表达不出来，已投影为 "
                f"mode={mc.motion_mode} base={mc.base_rpm} "
                f"→ 实际 (v={mc.v:.3f}, ω={mc.omega:.3f})。原因：{mc.note}",
                throttle_duration_sec=2.0)
        else:
            # 低速滑行下限：只提醒不改指令 —— 要不要抬速度是策略问题，
            # 车的物理行为改不了。阈值是交接包 F1 的实测值。
            floor = self.get_parameter("tracking_floor_rpm").value
            moving = [abs(x) for x in (mc.base_rpm, mc.base_rpm * mc.turn_inner_ratio_q8 / 256.0)
                      if abs(x) > 1e-6]
            if moving and min(moving) < floor:
                self.get_logger().warn(
                    f"内侧轮目标 {min(moving):.1f} RPM 低于跟踪下限 {floor} RPM，"
                    f"车实际会以约 {floor} RPM 滑行、转向比预期小",
                    throttle_duration_sec=3.0)
        return mc

    # ==================================================================
    # 上行：处理收到的帧
    # ==================================================================

    def _handle_frame(self, frame) -> None:
        kind, seq, payload = frame
        if kind == int(Kind.ACK):
            self.ack_count += 1
            info = parse_ack(payload)
            return
        if kind == int(Kind.TELEMETRY):
            self._on_telemetry(parse_telemetry(payload))
            return
        if kind == int(Kind.DIAGNOSTIC):
            self._on_diagnostic(parse_diagnostic(payload))
            return
        if kind == int(Kind.POSITION_TELEMETRY):
            info = parse_position_telemetry(payload)
            self.position_busy = info['busy']
            self.position_progress = info['progress_counts']
            # ★★ `done` 是**电平**，不是边沿。固件在段完成时置 `done=1`，
            #    要到**下一段 start** 才清零（chassis_position_controller.v:178
            #    清、:201/:225/:239/:258 置），而 0x84 是 20 Hz 周期广播的一部分
            #    （见 acg720_firmware._packet 的 'position' 分支）——
            #    所以段与段之间那几十毫秒里，同一个 `done=1` 会被报好几次。
            #    如果照单全收，驱动会把上一段的完成帧当成**新段**的结果，
            #    于是"还没走完就宣布走完"，车停在半路而且不报错。
            #    这里只认上升沿。本工程真的踩过这一条（HIL 的 [6] 转到 89° 就停）。
            rising = bool(info['done']) and not self._position_done_level
            self._position_done_level = bool(info['done'])
            self.position_done = bool(info['done'])
            if info['done']:
                self.position_result = info['result_code']
            if rising:
                self.get_logger().info(
                    f"位置动作完成：result={info['result_code']} "
                    f"progress={info['progress_counts']}/{info['target_counts']}")
                if self.segment_executor is not None:
                    # ★ 只**记下来**，真正的"发下一段"放到控制拍的发送路径上做：
                    #   串口写只允许有一个入口，否则限速和 SEQ 的记账会有两个说法。
                    self._pending_segment_result = (
                        int(info['result_code']), int(info['progress_counts']))
            return
        if kind == int(Kind.IMU):
            return                      # IMU 帧暂时只丢给 telemetry_raw 用量统计
        self.get_logger().warn(
            f"收到未定义命令字 0x{kind:02X}，payload={payload.hex(' ')}",
            throttle_duration_sec=5.0)

    def _on_telemetry(self, t: dict) -> None:
        self.telemetry_count += 1
        self.fw_state = SafetyState(t['state'])
        self.fw_fault = FaultReason(t['fault'])
        if self._needs_resync:
            # ★ 主机重启后 SEQ 归零不会被固件接受：必须先用 0x81 payload[2] 的
            #   last_command_seq 对齐，否则新序号会被判成重复而**静默丢弃**
            #   （现象：话题有、日志无错、车就是不动）。
            #   ⚠ 用显式标志而不是 "cmd_count == 0"：断线重连之后 cmd_count
            #     已经不是 0 了，但它**同样**需要重新对齐 —— HIL 测试抓到过这一条。
            #   ⚠ 对齐前必须先 `reset_input_buffer()` 丢掉积压的旧遥测，
            #     否则会按一帧过期的 last_command_seq 对齐。
            self._needs_resync = False
            self.seq.resync_from_telemetry(t['last_command_seq'])
            self.seq_synced = True
            self.get_logger().info(
                f"SEQ 已按固件上报对齐：下一条用 {self.seq.sequence}")

        if self.fw_state == SafetyState.FAULT:
            self._on_fault_latched()
        elif self._last_fault_logged is not None:
            self.get_logger().warn("底盘已从 FAULT 恢复（外部复位过），准备重新 ARM")
            self.motion_blocked = False
            self.block_reason = ""
            self.arm_sent = False
            self._last_fault_logged = None

        raw = String()
        raw.data = (
            f"n={self.telemetry_count} state={self.fw_state.name} "
            f"fault={self.fw_fault.name} last_cmd_seq={t['last_command_seq']} "
            f"target_rpm={t['target_rpm_abcd']} speed_rpm={t['speed_rpm_abcd']} "
            f"duty={t['duty_cycles_abcd']} channel_fault={t['fault_abcd']} "
            f"crc_errors={t['crc_errors']} duplicates={t['duplicates']} "
            f"frame_errors={t['frame_errors']}"
        )
        self._fw_counters = {
            'crc': t['crc_errors'],
            'dup': t['duplicates'],
            'frame': t['frame_errors'],
        }
        self.raw_pub.publish(raw)

    def _on_diagnostic(self, d: dict) -> None:
        """0x83：位流版本校验 + 里程计积分（累计计数是一手数据）。"""
        self.build_id = d['build_id']
        expected = str(self.get_parameter("expected_build_id").value or "").strip()
        if expected and self.build_id_ok is None:
            # build_id 就是 8 位十六进制（chassis_telemetry_imu_tx.v:40）
            want = int(expected, 16)
            self.build_id_ok = (d['build_id'] == want)
            if self.build_id_ok:
                self.get_logger().info(
                    f"位流版本校验通过：build_id=0x{d['build_id_hex']}")
                self._latch(f"build_id OK 0x{d['build_id_hex']}")
            else:
                self.motion_blocked = True
                self.block_reason = f"位流版本不符（板上 0x{d['build_id_hex']}，期望 0x{want:08X}）"
                self.get_logger().error(
                    f"★ 位流版本不符：板上 0x{d['build_id_hex']}，"
                    f"期望 0x{want:08X}。**拒绝发送任何运动命令。**"
                    f"多半是烧了 24_wifi_car_curve_model.fs 而不是位置环版本 —— "
                    f"请确认位流后重启本节点。")
                self._latch(f"build_id MISMATCH 板上=0x{d['build_id_hex']} 期望=0x{want:08X}")

        # ---- 里程计：s32 累计计数 → 增量 → 位姿 ----
        counts = d['position_count_abcd']
        now_ns = self.get_clock().now().nanoseconds
        if self._prev_counts is not None:
            if self.last_telemetry_time is None:
                dt = 0.05
            else:
                dt = max((now_ns - self.last_telemetry_time) / 1e9, 1e-4)
            delta = [delta32(counts[i], self._prev_counts[i]) for i in range(4)]
            left_counts = (delta[1] + delta[2]) / 2.0      # B + C
            right_counts = (delta[0] + delta[3]) / 2.0     # A + D
            dl = self.geom.counts_to_distance_m(left_counts)
            dr = self.geom.counts_to_distance_m(right_counts)
            d_center = (dl + dr) / 2.0
            d_yaw = (dr - dl) / self.geom.effective_separation_m
            self.pose = Pose2D(
                x=self.pose.x + d_center * math.cos(self.pose.yaw + d_yaw / 2.0),
                y=self.pose.y + d_center * math.sin(self.pose.yaw + d_yaw / 2.0),
                yaw=wrap_angle(self.pose.yaw + d_yaw),
            )
            self._publish_odom(d_center / dt, d_yaw / dt)
        self._prev_counts = list(counts)
        self.last_telemetry_time = now_ns

    def _publish_odom(self, v: float, omega: float) -> None:
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_yaw(self.pose.yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.pose.x
        odom.pose.pose.position.y = self.pose.y
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        # 协方差：先用保守估计。等实测出重复定位精度再改小，
        # 否则 AMCL 会过度信任里程计。
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.02
        self.odom_pub.publish(odom)

        if self.get_parameter("publish_tf").value:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.pose.x
            tf.transform.translation.y = self.pose.y
            tf.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            self.tf_broadcaster.sendTransform(tf)

    def _on_fault_latched(self) -> None:
        """底盘锁死。**必须吵**，并且立刻停止再发运动命令。"""
        if self._last_fault_logged == self.fw_fault:
            return
        self._last_fault_logged = self.fw_fault
        self.motion_blocked = True
        self.block_reason = f"底盘锁死：{self.fw_fault.name}"
        text = (f"★ 底盘已锁死：{self.fw_fault.name} —— {fault_cn(self.fw_fault)}。"
                f"本节点不会再发任何运动命令；FAULT 在固件里是**自锁**的，"
                f"只能断电，或「松开急停 S0 再按 S4」复位。")
        self.get_logger().error(text)
        self._latch(text)

    # ==================================================================
    # 订阅 / 诊断
    # ==================================================================

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.last_cmd_time = time.monotonic()
        self.last_cmd_v = float(msg.linear.x)
        self.last_cmd_w = float(msg.angular.z)
        if self.cmd_timed_out:
            self.cmd_timed_out = False
            self.get_logger().info("收到新的 /cmd_vel，恢复运动")
        # position 模式下 /cmd_vel 不是控制输入，而是**人工接管**信号：
        # 摇杆永远该能压过自主导航。台架调试时这是保命开关。
        if self.control_mode == "position" \
                and self.get_parameter("cmd_vel_overrides_goal").value:
            if abs(self.last_cmd_v) > 1e-3 or abs(self.last_cmd_w) > 1e-3:
                self._abort_goal(
                    f"收到非零 /cmd_vel（v={self.last_cmd_v:.3f} "
                    f"w={self.last_cmd_w:.3f}），人工接管")

    def _on_status_timer(self) -> None:
        parts = [
            f"connected={self.serial_connected}",
            f"opens={self._open_count}",
            f"mode={self.control_mode}",
            f"state={self.fw_state.name if self.fw_state is not None else 'N/A'}",
            f"fault={self.fw_fault.name}",
            f"build_id={'N/A' if self.build_id is None else f'0x{self.build_id:08X}'}",
            f"seq={self.seq.sequence}",
            f"telemetry={self.telemetry_count}",
            f"cmds={self.cmd_count}",
            f"acks={self.ack_count}",
            f"crc_errors={self._parser.crc_errors}",
            f"dropped_bytes={self._parser.discarded_bytes}",
            f"fw_crc_errors={self._last_fw_counter('crc')}",
            f"duplicates={self._last_fw_counter('dup')}",
            f"frame_errors={self._last_fw_counter('frame')}",
            f"motion_cmds={self.motion_cmd_count}",
            f"projected={self.projection_count}",
        ]
        if self.last_move is not None:
            parts.append("last_move=(mode={},base={},ratio={})".format(
                self.last_move.motion_mode, self.last_move.base_rpm,
                self.last_move.turn_inner_ratio_q8))
            parts.append("achieved=(v={:.3f},w={:.3f})".format(
                self.last_move.v, self.last_move.omega))
        if self.segment_executor is not None:
            # 分段导航的可观测性：目标、当前段、段计数、走过的距离、失败原因
            parts.append("segments=({})".format(
                self.segment_executor.describe().replace(" ", ",")))
            parts.append(f"position_busy={self.position_busy}")
            if self.segment_params is not None:
                parts.append(
                    f"seg_limits=(max_straight={self.segment_planner.max_counts_for_rpm(self.segment_params.straight_rpm)},"
                    f"max_turn={self.segment_planner.max_counts_for_rpm(self.segment_params.turn_rpm)})")
        if self.block_reason:
            parts.append(f"blocked=({self.block_reason})")
        if self.last_cmd_time is not None:
            parts.append(f"cmd_age={time.monotonic() - self.last_cmd_time:.2f}s")
        parts.append(f"pose=({self.pose.x:.3f},{self.pose.y:.3f},"
                     f"{math.degrees(self.pose.yaw):.1f}deg)")

        if self.last_telemetry_time is not None:
            age = self.get_clock().now().nanoseconds / 1e9 - self.last_telemetry_time
            parts.append(f"telemetry_age={age:.2f}s")
            if age > 0.5:
                self.get_logger().error(
                    f"{age:.2f}s 没有收到遥测帧——检查串口/波特率/接线")
        self.diag_pub.publish(String(data=" ".join(parts)))

    def _last_fw_counter(self, which: str) -> int:
        """固件自己记的接收侧计数器（0x81 遥测带回来的）。

        ⚠ 与 `crc_errors=`（**本机**解析器统计的）不是一回事：
          前者是"固件收到的坏帧"，后者是"我们收到的坏帧"。真机排查时
          两者要分开看 —— 只报一个数会指向错误的方向。
        """
        return self._fw_counters.get(which, 0)

    # ==================================================================

    def _latch(self, text: str) -> None:
        """发一条 latched 诊断。latched = 后连上来的订阅者也能看到，
        用来回答"这个节点上次启动到底成没成功"。"""
        self.diag_latched.publish(String(data=text))

    def destroy_node(self):
        # 退出前先停车再失能，否则下一次启动时电机会保持最后的指令
        try:
            if self.serial_connected and not self.dry_run:
                # ★ 只发一条 DISARM 就够了（DISARMED 会关掉电机使能），
                #   而且**必须**走正常的限速路径 —— 见 `_send` 的警告。
                # ★★ 必须**重试到发出去为止**：`_send` 会被命令限速挡掉，
                #    而"没发出去就关串口"的后果不是"少一条命令"，是
                #    **底盘留在 ARMED_STOP 没人喂狗 → 300 ms 后锁存
                #    FAULT_TIMEOUT → 只能断电复位**。用户只是 Ctrl-C 了一下。
                deadline = time.monotonic() + 0.3
                while time.monotonic() < deadline:
                    if self._send(Kind.DISARM, b"", "disarm"):
                        time.sleep(0.06)      # 让这一帧真的发出去再关串口
                        break
                    time.sleep(0.01)
        except (Exception, KeyboardInterrupt):
            # 退出路径被 Ctrl-C 打断是正常的（用户就是按了 Ctrl-C 才走到这里），
            # 不要因此甩一个 traceback 出来吓人。
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
