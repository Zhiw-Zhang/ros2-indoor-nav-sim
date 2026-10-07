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
