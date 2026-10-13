#!/usr/bin/env python3
"""ACG720 底盘运动学：Twist ↔ 四轮 RPM、编码器计数 ↔ 位姿增量。

真机迁移里最容易出隐蔽错误的一段就是这里，而且错了的症状是
"车在动、日志正常、但地图建歪 / 定位漂移"——很难现场查。
所以这一层是**纯函数**，不依赖 rclpy 也不依赖串口，可以直接单元测试。

三件必须分清的事
----------------
1. **几何轮距 b** = 200 mm（交接包：轮距，实测待复核）。
   它决定"物理上车转多快"。

2. **有效轮距 b_eff = b · wheel_separation_scale**。
   skid-steer 原地转时四个轮子横向刮擦，车体真实转角**小于**差速模型
   预测的转角。仿真里实测 odom/真值 = 1.36，也就是"编码器以为转了 1 圈，
   车实际只转了 0.73 圈"。

   ACG720 的 FPGA **不知道**这个系数（它的 `wheel_separation` 是几何值），
   所以这个标定量必须由上位机在**下发和上报两侧同时**打进去：

       下发：Δv = ω_cmd · b_eff   →  ω_true = ω_cmd      （控制增益对）
       上报：ω_odom = Δv / b_eff  →  ω_odom = ω_true     （里程计尺度对）

   ⚠ 这与仿真里把 `wheel_separation_scale` 塞进 DiffDrive 插件是**同一个数学**，
     但**位置不同**：仿真改插件，真机改本文件的 `scale` 参数。

   ⚠⚠ 真机的 scale **必须重量**，不能沿用仿真的 1.36。仿真 1.36 是 ODE 物理引擎
      + `wheel_mu_lat=0.5` 的产物，与真车轮胎/地面/负载都无关。
     真机初始值填 **1.0**（= 不做修正），按 `docs/real_robot_plan.md` 第 3 节的
     顺序标定：原地转固定时间 → scale = θ_odom / θ_true。

3. **速度 vs 位置**：速度环只需要 1 和 2；位置环（0x05）还额外依赖
   **CPR = 1320 counts/输出轴圈**，而 CPR 目前**也未实车复核**（交接包修正 2）。
   所以位置环精度在 CPR 复核前都不能算数。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

# ============================================================================
# 轮位约定
# ============================================================================
# 依据：交接包 top.v:4 的注释与 chassis_telemetry_imu_tx.v:254 的拼接顺序
#       {rear_right, rear_left, front_left, front_right}
# 于是四个通道的顺序是 A=右前、B=左前、C=左后、D=右后。
# 左侧 = B/C，右侧 = A/D。

#: 通道索引 → 名称（索引顺序就是串口载荷里的顺序）
WHEEL_ORDER = ("A_right_front", "B_left_front", "C_left_rear", "D_right_rear")
WHEEL_A_RIGHT_FRONT = 0
WHEEL_B_LEFT_FRONT = 1
WHEEL_C_LEFT_REAR = 2
WHEEL_D_RIGHT_REAR = 3


def wheels_to_lr(a: float, b: float, c: float, d: float) -> Tuple[float, float]:
    """四轮 RPM → (左轮平均 RPM, 右轮平均 RPM)。

    用交接包 4.4 节的归一化反推：左 = (B+C)/2，右 = (A+D)/2。
    返回的是**输出轴 RPM**，不是 m/s。

    ⚠ 这个函数原先住在 `acg720_protocol.py` 里。它是**运动学**，不是协议，
      所以迁到了本文件 —— 协议层换掉时它不该跟着陪葬。
    """
    return ((b + c) / 2.0, (a + d) / 2.0)


def lr_to_wheels(left: float, right: float) -> Tuple[float, float, float, float]:
    """(左轮, 右轮) RPM → 四轮目标 (A, B, C, D)。

    轮位 A=右前、B=左前、C=左后、D=右后。同侧两轮给相同目标 ——
    这与交接包里"一侧两轮同目标"的曲线工况一致（F1 验收的工况表就是
    按 (A,B,C,D) 给的，例如右弯 40RPM 给 30/40/40/30，左右各自相等）。
    """
    return (right, left, left, right)


@dataclass
class SkidSteerGeometry:
    """底盘几何与标定量。默认值全部取自交接包，但**都标注了可信度**。

    默认值来源：`小车数据总览_实测状态分级.md` 第 4.1 / 4.2 节。
    """

    #: 轮径 65 mm → 半径 32.5 mm。【待确认】UNVALIDATED_GEOMETRY
    wheel_radius_m: float = 0.0325
    #: 轮距 200 mm。【待确认】未实车标定
    wheel_separation_m: float = 0.200
    #: 轴距 185 mm。【待确认】未实车标定。速度环用不到，仅记录 + 将来给 Nav2
    wheelbase_m: float = 0.185
    #: 每输出轴一圈的编码器计数。【待确认】交接包修正 2：1320 未转整数圈复核
    counts_per_rev: float = 1320.0
    #: ★ skid-steer 有效轮距系数。真机必须重量，初始 1.0
    scale: float = 1.0

    # ---- 派生量 ----

    @property
    def effective_separation_m(self) -> float:
        """有效轮距 b_eff = b · scale。下发和上报都用它。"""
        return self.wheel_separation_m * self.scale

    @property
    def wheel_circumference_m(self) -> float:
        return 2.0 * math.pi * self.wheel_radius_m

    @property
    def counts_per_meter(self) -> float:
        """每米行驶对应的编码器计数。距离换算的唯一尺度。"""
        return self.counts_per_rev / self.wheel_circumference_m

    @property
    def meters_per_count(self) -> float:
        return 1.0 / self.counts_per_meter

    @property
    def rpm_per_mps(self) -> float:
        """输出轴 RPM ↔ 轮面线速度 m/s 的换算系数。

        rpm = v / (2πr) · 60     →     rpm_per_mps = 60 / (2πr)
        默认 (r=0.0325)：≈ 293.9 RPM per m/s。
        """
        return 60.0 / self.wheel_circumference_m

    def rpm_to_mps(self, rpm: float) -> float:
        return rpm / self.rpm_per_mps

    def mps_to_rpm(self, v: float) -> float:
        return v * self.rpm_per_mps

    # ---- 上报方向：轮速 → 车体速度 ----

    def wheels_to_twist(self, left_rpm: float, right_rpm: float) -> Tuple[float, float]:
        """左右轮 RPM → (线速度 m/s, 角速度 rad/s)。

        角速度用**有效轮距**，这样 ω_odom 与车体真实转角一致（见模块 docstring）。
        """
        v_l = self.rpm_to_mps(left_rpm)
        v_r = self.rpm_to_mps(right_rpm)
        v = (v_l + v_r) / 2.0
        omega = (v_r - v_l) / self.effective_separation_m
        return v, omega

    # ---- 下发方向：车体速度 → 轮速 ----

    def twist_to_wheels(self, v: float, omega: float) -> Tuple[float, float]:
        """(线速度 m/s, 角速度 rad/s) → (左轮 RPM, 右轮 RPM)。

        用**有效轮距**：Δv = ω · b_eff，于是车体真实角速度 ≈ ω_cmd。
        """
        half = omega * self.effective_separation_m / 2.0
        return (self.mps_to_rpm(v - half), self.mps_to_rpm(v + half))

    # ---- 位置：编码器计数 ↔ 距离 / 转角 ----

    def counts_to_distance_m(self, counts: float) -> float:
        """计数 → 轮面走过的距离（米）。"""
        return counts * self.meters_per_count

    def distance_to_counts(self, meters: float) -> float:
        return meters * self.counts_per_meter

    def counts_to_rotation_rad(self, left_counts: float, right_counts: float) -> float:
        """左右轮计数差 → 车体转角（弧度），用有效轮距。"""
        d_l = self.counts_to_distance_m(left_counts)
        d_r = self.counts_to_distance_m(right_counts)
        return (d_r - d_l) / self.effective_separation_m

    # ---- 0x05 POSITION 的目标计数 ----
    #
    # ★ 口径来自 RTL，不是猜的：`chassis_position_controller.v:78` 是
    #       progress = (归一化四轮增量之和) >> 2
    #   也就是**四个轮子增量的平均**，不是求和。归一化（:117-141）对前进是
    #   四轮取正、对原地转是"右前/右后取正、左前/左后取负"。
    #   两种动作算下来都等于"**每个轮子滚过的计数**"，所以下面两个函数共用
    #   同一个 counts_per_metre 尺度。

    def distance_to_position_counts(self, meters: float) -> float:
        """要走 `meters` 米，0x05 POSITION 的 target_counts 填多少。"""
        return self.distance_to_counts(meters)

    def position_counts_to_distance_m(self, counts: float) -> float:
        return self.counts_to_distance_m(counts)

    def rotation_to_position_counts(self, radians: float) -> float:
        """要原地转 `radians` 弧度，target_counts 填多少。

        原地转时左右轮反向各滚 s 米，转向中心在两轮中点，所以
        `Δθ = 2s / L_eff`，即 `s = Δθ·L_eff/2`。
        ⚠ 这里必须用**有效（打滑后）轮距**：skid-steer 原地转时四个轮子横向
        刮擦，车体真实转角小于几何预测。用手指拨一圈比几何值可靠。
        """
        return self.distance_to_counts(abs(radians) * self.effective_separation_m / 2.0)

    def position_counts_to_rotation_rad(self, counts: float) -> float:
        return self.counts_to_distance_m(counts) * 2.0 / self.effective_separation_m


# ============================================================================
# 0x02 MOVE：当前固件唯一的连续运动命令，与它的表达能力边界
# ============================================================================
#
# MOVE 的参数是 `(motion_mode, base_rpm, turn_inner_ratio_q8)`：
#
#     chassis_motion.v:32-36
#         target_valid = (base_rpm <= 200) && (motion_mode <= 6)
#         outer_rpm    = base_rpm
#         inner_rpm    = base_rpm * ratio >> 8        ← **无符号**，只能缩小
#
# 所以：
#   mode 1/2 前进/后退：四轮同速
#   mode 3/4 左转/右转：外侧 = base，内侧 = base×ratio/256
#   mode 5/6 原地左/右旋：左右反向同速
#
# ★ 它**能**表达的范围比"只能直行和原地转"大得多：`(base, ratio)` 是两个自由
#   参数，正好解两个未知数 (v, ω)。同号两轮时
#       ratio = (v − |ω|·L/2) / (v + |ω|·L/2)
#   对任意转弯半径 `R = v/ω ≥ L/2` 都可解；`ratio = 0` 就是"内侧轮停转、
#   绕内侧轮转"，这也是**不反转内侧轮能达到的最紧弯**。
#
# ★ 表达不出来的只有两类，本模块会把它们**投影**到最近的可达动作并标记
#   `achievable=False`（绝不静默发一个固件会理解成别的动作的目标）：
#     ① 转弯半径 < L/2（内侧轮需要反转）
#     ② 倒退转弯（内侧轮同样需要反转；固件没有倒退弧线这种动作）
#
# ⚠ 与 4B 的关系：即使 (v, ω) 能解成 MOVE，**3.0 秒连续 RUNNING 上限**
#   （chassis_safety_fsm.v:11,57-58,72-77）仍然会让连续导航在 3 秒后锁死底盘。
#   所以本模块解出来的是"一旦 FPGA 放开那条上限就能用"的能力，
#   今天它的用途是台架验证与给 FPGA 变更文档提供量化依据。

#: 转弯半径的下限系数：最小半径 = 该系数 × 有效轮距。0.5 = 半轮距（RTL 硬下限）。
MIN_TURN_RADIUS_FACTOR = 0.5


@dataclass
class MoveCommand:
    """一次 MOVE 的参数，外加"这组参数实际会跑出什么"。

    存在的意义是让**投影**可见：请求 (v, ω) 之后如果 `achievable=False`，
    `v`/`omega` 是被投影后的实际值、`note` 说明丢掉了什么。真机上
    "静默地把转向吃掉"是最难查的一类问题。
    """

    motion_mode: int
    base_rpm: int
    turn_inner_ratio_q8: int
    achievable: bool
    v: float
    omega: float
    note: str = ""


def move_to_twist(motion_mode: int, base_rpm: int, turn_inner_ratio_q8: int,
                  geom: "SkidSteerGeometry") -> Tuple[float, float]:
    """(mode, base_rpm, ratio) → 车体**实际**会得到的 (v, ω)。

    正解走的是固件自己的展开逻辑（`chassis_motion`），所以这里算出来的
    "实际值"与固件真会做的事逐条一致 —— 驱动用它打印 `achieved=` 诊断。
    """
    from acg720_firmware import WHEEL_A_RIGHT_FRONT as _A
    from acg720_firmware import WHEEL_B_LEFT_FRONT as _B
    from acg720_firmware import chassis_motion as _expand

    wheels = _expand(int(motion_mode), int(base_rpm), int(turn_inner_ratio_q8))
    left = (wheels[_B] + wheels[2]) / 2.0
    right = (wheels[_A] + wheels[3]) / 2.0
    return geom.wheels_to_twist(left, right)


def twist_to_move(v: float, omega: float, geom: "SkidSteerGeometry",
                  max_base_rpm: int = 200) -> MoveCommand:
    """(v, ω) → 最接近的一组 MOVE 参数。

    三种情形（见本文件上方那段注释的推导）：

    | 左右轮目标 | 用哪个动作 | 是否精确 |
    |---|---|---|
    | 都 ≥ 0 | mode 3/4 弧线（两轮同速时退化成 mode 1 直线） | 是 |
    | 都 ≤ 0 | mode 2 直退，**丢掉 ω**（保留请求的 v） | 否，倒退转弯不可达 |
    | 异号 | mode 5/6 原地转，**丢掉 v**（保留请求的 ω） | 否，半径 < L/2 不可达 |

    ★ 投影时**保留哪个量是有讲究的**：两种不可达情形都保留"更接近请求的那一半"，
      而不是"按峰值轮速推算" —— 后者会让车比命令的更快，是危险的方向。
      `move_to_twist()` 反算出来的 `v`/`omega` 就是投影后的实际值。

    `base_rpm` 与 `ratio` 都量化过（整数 RPM、1/256 比例），所以即使
    `achievable=True` 也有一点点量化误差 —— 用 `move_to_twist()` 即可看到。
    """
    max_base_rpm = int(max_base_rpm)
    if max_base_rpm <= 0:
        raise ValueError("max_base_rpm 必须为正")

    def build(mode: int, base: int, ratio: int, ok: bool, note: str) -> MoveCommand:
        got_v, got_w = move_to_twist(mode, base, ratio, geom)
        return MoveCommand(mode, base, ratio, ok, got_v, got_w, note)

    left_rpm, right_rpm = geom.twist_to_wheels(v, omega)
    if max(abs(left_rpm), abs(right_rpm)) < 1e-6:
        return build(0, 0, 255, True, "")

    # ---- 两个轮子都朝前：弧线可以精确表达 ----
    if left_rpm >= 0.0 and right_rpm >= 0.0:
        base = int(round(max(left_rpm, right_rpm)))
        if base > max_base_rpm:
            return build(0, 0, 255, False,
                         f"需要 {base} RPM 超过上限 {max_base_rpm}；应先对 (v, ω) 限幅")
        if base == 0:
            return build(0, 0, 255, True, "")
        if abs(left_rpm - right_rpm) < 1e-9:
            return build(1, base, 255, True, "")
        ratio = max(0, min(255, int(round(min(left_rpm, right_rpm) / base * 256.0))))
        mode = 3 if left_rpm < right_rpm else 4       # 内侧轮决定左转还是右转
        return build(mode, base, ratio, True, "")

    # ---- 两个轮子都朝后：只能直退，ω 表达不出来 ----
    if left_rpm <= 0.0 and right_rpm <= 0.0:
        # 丢掉 ω、保留 v：投影后的直线速度就等于命令速度
        straight = geom.mps_to_rpm(abs(v))
        base = max(1, min(max_base_rpm, int(round(straight))))
        note = ""
        if abs(omega) > 1e-6:
            note = "固件没有倒退转弯（mode 2 只有直退）：ω 被丢弃"
            if base >= max_base_rpm and straight > max_base_rpm:
                note += f"，且 |v|={abs(v):.3f} m/s 也超过上限被限幅"
        return build(2, base, 255, abs(omega) <= 1e-6, note)

    # ---- 左右异号：半径 < L/2，内侧轮需要反转 ----
    # 投影成原地转：保留 ω（丢掉 v）。base 由 |ω|·L/2 折算。
    spin = geom.mps_to_rpm(abs(omega) * geom.effective_separation_m / 2.0)
    base = max(1, min(max_base_rpm, int(round(spin))))
    mode = 5 if omega > 0 else 6
    note = ""
    if abs(v) > 1e-6:
        note = (f"转弯半径小于 {geom.effective_separation_m / 2.0:.3f} m"
                f"（内侧轮需要反转）→ 投影成原地转，v 被丢弃")
    return build(mode, base, 0, abs(v) <= 1e-6, note)


def wrap_angle(a: float) -> float:
    """把角度归一化到 (-π, π]。

    ⚠ 这里不能图省事写成 `(a + π) % 2π − π`。那个写法在 a 正好等于 π 时
    会返回 **−π**（因为 π 会落进下取整那一侧），于是同一句 `wrap_angle(π)`
    既可能给 +π 也可能给 −π。这不是纯粹的美学问题：这个函数的输出直接进
    `/odom` 的 yaw 和 TF，而 ±π 的跳变会让 RViz 里车头瞬间翻转、
    也会让 AMCL 的初始位姿差整整 2π。

    所以显式处理边界，保证 π 只会映射到 +π。
    """
    a = math.fmod(a, 2.0 * math.pi)          # (-2π, 2π)，保留符号
    if a > math.pi:
        a -= 2.0 * math.pi
    elif a <= -math.pi:
        a += 2.0 * math.pi
    return a


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """绕 z 轴的偏航 → 四元数 (x, y, z, w)。"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def integrate(self, v: float, omega: float, dt: float) -> "Pose2D":
        """按 (v, ω) 推进 dt 秒。

        用中点积分（先转一半、再走、再转一半）。对 50 ms 的采样周期来说，
        欧拉积分和它的差别在毫米级，但原地旋转时欧拉积分会累积明显误差，
        而原地旋转正是室内导航最常用的动作，所以这里用中点法。
        """
        yaw_mid = self.yaw + omega * dt / 2.0
        return Pose2D(
            x=self.x + v * math.cos(yaw_mid) * dt,
            y=self.y + v * math.sin(yaw_mid) * dt,
            yaw=wrap_angle(self.yaw + omega * dt),
        )
