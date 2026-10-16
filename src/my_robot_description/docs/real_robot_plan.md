# 实车迁移与验证计划

这份文档回答一个问题：**仿真里已经跑通的三条功能，搬到真车上还要做哪些事。**

仿真侧的验收结论见 `acceptance.md`。**参数怎么量**见 `real_robot_params.md`——那份是
逐项的标定操作手册，本文不重复，只给出「谁先谁后」的顺序和它与其他工作的依赖关系。

> 先说清本文的性质：下面的内容**大部分是读代码 + 工程判断得出的，不是实车实测结果**。
> 凡是没有实测支撑的地方，文中都会标出来。目前**没有**任何真机数据。

---

## 0. 结论速览

| 工作块 | 现状 | 工作量 |
|---|---|---|
| 导航层（Nav2 / SLAM / 参数 / 地图 / 路点） | ✅ 与仿真解耦，可直接复用 | 无需重做 |
| **底盘驱动层（ACG720 串口 / /odom / TF）** | **✅ 已做完，且无硬件即可验证** | 剩余：核实 payload 布局 |
| 参数标定 | ⚠️ 清单已备好，值全是占位 | 中等，但顺序不能错 |
| **验证手段** | ❌ **现在的方法真机上失效** | 被低估的一块 |
| 安全措施 | ❌ 仿真里不存在这个概念 | 必须补齐才能上车 |

三条功能里，**① 起点到终点** 和 **③ 路径点** 搬到真机主要是标定问题；
**② 障碍物避让** 最麻烦，因为**"零接触"这个结论在真机上没法用现在的工具证明**（见第 4 节）。

> **本次更新：底盘驱动层按 FPGA 源码重做完了。** 详见
> [`acg720_protocol.md`](acg720_protocol.md)。要点：
>
> * 真协议 V1：`A5 5A | VER | KIND | SEQ | LEN | PAYLOAD | CRC16`，
>   CRC-16/CCITT-FALSE，7 条命令、5 种遥测。**每条结论都标了 RTL 的 `文件:行号`**，
>   并与厂商的 `chassis_control.py` 做了逐字节交叉核对。
> * `acg720_firmware.py`：按 RTL 逐条复刻的**固件行为模型**
>   （帧拒绝、安全状态机、3.0 s 上限、故障自锁、ACK 单槽、位置环）。
> * `acg720_driver`：真协议驱动节点，启动即校验位流 `build_id`，故障必吼。
> * `acg720_simulator`：PTY 虚拟底盘，接上面那个模型，**没有硬件也能撞到真问题**。
> * `position_segments.py`：**分段导航**的规划器与执行状态机（纯逻辑）。
> * 测试：349 例纯函数 + 59 项 HIL + 65 项分段导航 HIL + 32 项 launch 冒烟。
>
> ⚠ **第一阻塞项已降级为"性能问题"**：固件的 3.0 s 运行上限挡的是
> **连续的 `0x02 MOVE`**（跑满 3.0 秒就锁存 `FAULT_RUN_LIMIT`，只能断电或
> "松开 S0 再按 S4"复位；**重发命令不能续期**，10 Hz 重发一样触顶）。
> 但位置环**不受**这条限制（`chassis_safety_fsm.v:57-58` 的 `!position_active`），
> 所以 `control_mode:=position` 用一串 `0x05 POSITION` 段**不改固件就能连续导航** ——
> 实测单段连续跑 **8.26 s** 而 `fault=NONE`、2.5 m 目标走到 2.498 m。
> 代价是**没有圆弧**（只有"转—直行—转"折线）且平均速度约 0.09 m/s。
> 用法见 [`position_navigation.md`](position_navigation.md)。
> 想要连续圆弧运动才需要 FPGA 变更，见
> [`fpga_change_request.md`](fpga_change_request.md) —— **只需改一处**。
>
> ⚠ **第二阻塞项**：雷达型号还没定。用户已确认会加 2D 雷达（LD19 / RPLIDAR A1 级），
> 但具体型号、驱动包、安装位姿都还没有，所以 `robot.launch.py` 默认
> `use_laser:=false` 且**不绑定任何雷达型号**。

---

## 1. 已经现成、不用重做的

### 1.1 导航层与仿真已经解耦

`launch/nav.launch.py:170-180` 用 `IfCondition(sim)` 把 Gazebo 的 include 包起来：

```python
actions.append(GroupAction(
    condition=IfCondition(sim),          # ← sim:=false 时整块跳过
    launch_configurations={'rviz': 'false'},
    actions=[IncludeLaunchDescription(... gazebo_sim.launch.py ...)],
))
```

Nav2、SLAM、地图、路点、参数文件全都在这一层之上，不依赖 Gazebo。所以
`sim:=false` 这个入口是**设计时就留好的**，不是要新加的功能。

### 1.2 标定清单已经写好

`real_robot_params.md` 已包含：

- URDF 顶部 6 组 `<xacro:property>` 占位值 + 每一项的**测量方法**
- `wheel_separation_scale` 的完整标定步骤与仿真实测数据（仿真量出来 1.36）
- **连带修改对照表**：改了 URDF 的哪一项，Nav2 的哪些参数必须跟着改
- 改完的自检命令（几何一致性、TF 抽查、里程计尺度验证）

### 1.3 可以直接搬走的东西

| 类型 | 文件 |
|---|---|
| Nav2 参数 | `config/nav2_params.yaml` |
| SLAM 参数 | `config/slam_toolbox.yaml` |
| 地图 | `maps/*.yaml` + `*.pgm` |
| 路点路线 | `routes/*.txt` |

**但注意**：`maps/` 里的地图是**仿真场地**的。真场地要重新建图，这几个文件只是格式参考，
不是能直接用的数据。

---

## 2. 驱动层：必须新做（最大的一块）

现在仿真里 `/scan`、`/odom`、`/tf` 全部由 Gazebo 插件产生：

| 位置 | 现在（仿真） | 真机要换成 |
|---|---|---|
| `urdf/my_robot.urdf.xacro:217` | `<plugin filename="gz-sim-diff-drive-system">` | 底盘驱动节点（串口 / CAN / micro-ROS），发布 `/odom` 并广播 `odom → base_link` |
| `urdf/my_robot.urdf.xacro:239` | `<sensor type="gpu_lidar">` | 雷达驱动节点，发布 `/scan` |
| `launch/gazebo_sim.launch.py:122` | `ros_gz_bridge` 桥 `/world/default/dynamic_pose/info` | 整块不需要（那是真值位姿，真机没有） |
| `launch/gazebo_sim.launch.py:114` | 桥 `/clock` | 不需要，`use_sim_time` 应为 `false` |

### 2.1 硬约束：名字不能改

这一条如果做错，`nav2_params.yaml` 里几十处要跟着改，而且症状是"地图建歪 / 车贴墙"这种
很难查的表现。所以：

| 类别 | 必须保持的名字 |
|---|---|
| Topic | `/scan`、`/odom`、`/cmd_vel`（Nav2 实际发的是 `/cmd_vel_smoothed`） |
| Frame | `odom`、`base_link`、`laser_frame` |
| TF 链 | `map → odom`（AMCL 发）、`odom → base_link`（驱动发）、`base_link → laser_frame`（URDF 发） |

`robot_state_publisher` 继续用同一份 URDF（去掉 gz 插件和 sensor 之后），这样
`base_link → laser_frame` 的几何关系不用重配。

### 2.2 顺带要修的几处 —— ✅ 已改完

原先读代码发现的、`sim:=false` 时会碍事的三处，**现在都已经改掉并验证过**：

| 位置 | 原问题 | 现在 |
|---|---|---|
| `nav.launch.py` 194/224/262 | `use_sim_time` 三处硬编码 `'true'` | ✅ 提成独立 launch 参数 `use_sim_time`，留空时跟随 `sim`；仿真/真机/HIL 三种组合都能跑 |
| `nav.launch.py` 162 | 无条件检查 `worlds/<world>` 是否存在 | ✅ 改成 `sim_on` 时才检查。真机上传 `world` 无意义，不该因此启动失败 |
| `nav.launch.py` 138 | `world` 参数在非仿真下无意义 | ✅ 保留但跳过校验 |

**为什么 `use_sim_time` 必须跟 `sim` 分开**：真机上 `sim:=false` 只是"不启动
Gazebo"，此时**没有 `/clock`**。如果 Nav2 仍开着 `use_sim_time=true`，它会一直等
一个永远不来的时钟——症状是节点全起、TF 树完整、**车就是不动**，而且日志里
几乎没有报错。这是最难现场排查的一类问题，所以现在默认值紧跟 `sim`。

### 2.3 bringup launch —— ✅ 已实现

`launch/robot.launch.py` 已经写好并跑通（`test_bringup_smoke.py` 用真实
`ros2 launch` 验证）：

```
robot.launch.py
  ├── acg720_driver            底盘驱动（串口 → /odom + odom→base_link）
  ├── robot_state_publisher    URDF → base_link→laser_frame 等固定 TF
  ├── 雷达驱动（use_laser:=true 时，本工程不绑定型号）
  └── nav.launch.py sim:=false Nav2；use_sim_time 自动为 false
```

用法与各参数见 [`acg720_protocol.md`](acg720_protocol.md) 第 8 节与第 11 节。

> ⚠ **`odom → base_link` 只能有一个发布者。** 驱动节点发这条；URDF 里
> **不能**再把它写成关节（真车 URDF 的根是 `base_link`，不是 `odom`）。
> 两边都发会出现 `TF_REPEATED_DATA`，RViz 里车会抖。
> 冒烟测试专门断言了 `/tf` 的发布者数量 ≤2（驱动 + robot_state_publisher，
> 后者只在 `/tf_static` 上发固定关节）。

---

## 3. 参数标定：顺序不能错

**详细操作见 `real_robot_params.md`**，这里只讲顺序和依赖，因为顺序错了会白调。

```
① 轮子几何            wheel_radius / wheel_offset_y
      │                  推车走 1 m，看编码器报多少
      ↓
② wheel_separation_scale      原地转固定时间，比编码器积分转角 vs 实际转角
      │                  ★ 文档标红：「Nav2 定位精度的第一决定因素」
      ↓
③ 雷达安装位姿         lidar_mount_x/y/z
      │                  tf2_echo odom laser_frame 核对
      ↓
④ 雷达量程 / 噪声      lidar_range_* / lidar_noise_stddev
      │                  影响 AMCL 观测模型；仿真填 0.01，实车远大于 1 cm
      ↓
⑤ inflation_radius     ★ 最后调，因为它的正确取值依赖 ①②③④ 的结果
      ↓
⑥ 速度 / 加速度上限
```

### 关于第 ⑤ 项，必须强调

`inflation_radius` 应该约等于「外接圆半径 + 定位误差 + 余量」。

- **仿真里**：外接圆 0.273，取 0.35（余量约 8 cm）
- **真机上**：① ② ③ 没做完之前，你**不知道定位误差有多大**，所以这项没法提前定

⚠️ 而且真机上的 `inflation_radius` 只会**更大**（要兜住定位误差），所以**能过的门只会更宽**。

**不要拿仿真里 0.726 m 能过当参考。** 那一轮的事故根因就是膨胀半径要 0.70 m 而缝只有
0.726 m——差 2.6 cm 的余量。真机上这个余量会被定位误差吃光。详见 `nav2.md` 的
「能过 ≠ 好过：膨胀半径与通道宽度的关系」。

---

## 4. 最大的坑：验证手段在真机上失效

这一条最容易被低估。仿真里报告的那些 ✅，**很大一部分是靠 Gazebo 真值位姿给出的**，
而真机上没有这个东西。

### 4.1 现有工具的移植性

| 工具 | 真机 | 原因 |
|---|---|---|
| `scripts/cost_slice.py` | ✅ 可用 | 只订阅代价地图，与平台无关 |
| `scripts/nav_goals.py` | ✅ 可用 | 只发 action、读 `/amcl_pose`；文件头明确写了不依赖 `use_sim_time` |
| `scripts/mapeval.py` | ❌ **失效** | 拿 world SDF 当"标准答案"算占用格偏差，真机没有标准答案 |
| `scripts/obs_test.py --static` | ❌ **失效** | 订阅 `/world/default/dynamic_pose/info`（SceneBroadcaster 的真值位姿） |
| `scripts/xwd2png.py` | ❌ 失效 | 截 Gazebo GUI |

### 4.2 后果

**②「障碍物避让零接触」这个结论，真机上用现在的方法无法证明。**

仿真里的判定口径是：Gazebo 真值位姿 + 障碍物 OBB 的分离轴判定（不用 AABB，因为
`obs_roomB_slant` 转了 25°，AABB 面积是实体的 2.8 倍，会误报）。这套东西真机上没有对应物。

### 4.3 三条替代路线

| 方案 | 精度 | 成本 | 说明 |
|---|---|---|---|
| 动捕（Vicon / OptiTrack） | 最高 | 最高 | 有现成设备就直接用 |
| 天花板相机 + 车顶 AprilTag | 高 | 中 | **性价比最高**，需要自己标定相机内外参 |
| 地面贴格 + 人工卷尺 | 低 | 最低 | 一次只能量一个点，但**今天就能用** |

在补上其中任意一条之前，② 只能退化成"我看着它没撞"——**那不算验证**，不能写进验收报告。

---

## 5. 安全（仿真里撞了不要钱，真机不是）

| 项 | 要求 |
|---|---|
| 急停 | 车上有物理急停按钮 |
| 第二人 | 另一个人拿**无线急停**，全程盯着 |
| 初速 | 从 **0.15 m/s** 起步。这轮仿真的 0.5 m/s 不要直接用 |
| `collision_monitor` | 从"兜底"升级为"必须真的停" |
| 测试顺序 | **反过来**：先空旷场地 → 再软障碍（泡沫/纸箱）→ 最后才窄缝和细杆 |

⚠️ `collision_monitor` 这一项有具体理由：**仿真这三轮里它一次都没介入过**（因为规划器
一开始就把路绕开了）。也就是说这块**从未被验证过**，不能当作已有的安全网。

⚠️ 别一上来就复刻 `obs_roomB_slant` 那条 0.726 m 的缝。那是仿真里已知的失效点。

---

## 6. 分阶段推进与通过判据

| 阶段 | 内容 | 通过判据 | 前置 |
|---|---|---|---|
| 0 | 轮子架空：验转向符号、急停、TF 树完整 | 前后左右命令方向都对；`tf2_echo` 链完整 | 驱动层完成 |
| 1 | 里程计标定（直线 + 原地转） | 走 1 m 误差 <2%；转 360° 误差 <3° | 阶段 0 |
| 2 | 建图 | 门洞不糊、墙不重影 | 阶段 1 |
| 3 | 定位复现性（开回若干固定点） | 同一位置重复定位偏差 <5 cm | 阶段 2 |
| 4 | **①** 空旷场地点到点，慢速 | 到达 + 终点误差 <15 cm | 阶段 3 |
| 5 | **③** 路点 | 全部到达 | 阶段 4 |
| 6 | **②** 静态障碍物，由宽到窄 | 零接触 + **外部真值确认** | 阶段 4 + 第 4 节的真值方案 |
| 7 | 动态障碍（人走动） | —— | 阶段 6 |

判据里的数字（<2%、<3°、<5 cm、<15 cm）是**工程经验值，不是本项目实测出来的阈值**，
可以按实际硬件放宽或收紧。

阶段 4-6 的**测试用例可以直接沿用仿真那三轮**（见 `acceptance.md`），包括
`routes/rooms_obstacles_waypoints_a.txt` 那 5 个路点——只要真场地的布局能对上。

---

## 7. 文件改动总览

真机落地时预计要动的文件（**标 ✅ 的已经做完了**）：

| 文件 | 动作 | 状态 |
|---|---|---|
| `src/acg720_protocol_v1.py` | **新增**：真协议帧编解码（与厂商实现逐字节交叉核对） | ✅ 已做 |
| `src/acg720_firmware.py` | **新增**：按 RTL 逐条复刻的固件行为模型 | ✅ 已做 |
| `src/position_segments.py` | **新增**：分段导航的规划器 + 执行状态机（纯逻辑，无 ROS） | ✅ 已做 |
| `src/drive_kinematics.py` | Twist ↔ 四轮 RPM、计数 ↔ 位姿、MOVE 反解 | ✅ 已做 |
| `src/acg720_driver.py` | **重写**：底盘驱动节点（真协议 V1） | ✅ 已做 |
| `src/acg720_simulator.py` | **重写**：PTY 虚拟底盘，接固件行为模型 | ✅ 已做 |
| `docs/fpga_change_request.md` | **新增**：给 FPGA 的变更需求 | ✅ 已做 |
| `docs/position_navigation.md` | **新增**：分段导航（不改固件的连续移动） | ✅ 已做 |
| `urdf/acg720_real.urdf.xacro` | **新增**：真车 URDF（无 gz 插件/sensor） | ✅ 已做，几何值待实车复核 |
| `launch/robot.launch.py` | **新增**：真机 bringup | ✅ 已做 |
| `launch/nav.launch.py` | `use_sim_time` 提成参数；world 校验加条件 | ✅ 已改 |
| `test/`（七个文件） | 349 例单测 + 59 项 HIL + 65 项分段导航 HIL + 32 项冒烟 | ✅ 已做 |
| `docs/acg720_protocol.md` | **新增**：底盘驱动层文档 | ✅ 已做 |
| `config/nav2_params.yaml` | **改**：`footprint`、`inflation_radius`、速度/加速度上限、AMCL 雷达参数 | ⏳ 待实车标定 |
| `config/slam_toolbox.yaml` | **改**：`max_laser_range`、`minimum_time_interval` | ⏳ 待雷达型号确定 |
| `maps/*` | **替换**：真场地重新建图 | ⏳ 待雷达 |
| `scripts/mapeval.py` | **改或新增**：现在依赖 world SDF，真机需要不依赖真值的替代口径 | ⏳ 待第 4 节真值方案 |
| `scripts/obs_test.py` | **改或新增**：同上，真机版要接外部真值源 | ⏳ 待第 4 节真值方案 |
| `docs/real_robot_params.md` | **填**：把占位值换成实测值 | ⏳ 待实测 |
| 新增 `scripts/` | 可能：外部真值（AprilTag / 动捕）的接入脚本 | ⏳ |

**不需要改**：`cost_slice.py`、`nav_goals.py`、`routes/*`。

⚠️ **新增文件后必须重新 `colcon build --symlink-install`。** `install/` 里是**按文件**建的
符号链接（`real_robot_plan.md -> .../src/.../docs/real_robot_plan.md`），不是整目录链接，
所以 CMake 的 `DIRECTORY` 规则虽然涵盖了它，也要跑一次 build 才会生成链接。
（本文档本身就撞过这个：写完 `ls install/.../docs/` 里没有，build 一次才出现。
`real_robot_params.md` 开头也记了同一条。）

⚠️ **还有一个更隐蔽的坑：`install(PROGRAMS)` 会照抄源码的权限位。**
新增的 `acg720_driver.py` / `acg720_simulator.py` 如果源码没有 `+x`，
装出来的符号链接也就没有 `+x`，`ros2 launch` 会直接报
`executable 'acg720_driver.py' not found on the libexec directory`
——**看起来像"文件没装进去"，实际是权限问题**。本项目真的踩过这一条。
`test_bringup_smoke.py` 的第 0 步专门检查这个，别再手工排查。

---

## 8. 阻塞项：先确认硬件

原来这里列了五条"不确认就没法动手"的问题。**其中三条已经答掉了**：
协议布局拿到了 FPGA 源码并逐行核对过；算力平台定了（香橙派 5 Plus +
Docker 里的 Jazzy）；CPR 也在 RTL 注释里确认了。

### ✅ 已经答掉的

* **协议布局**：不再是"本工程自行定义"。真协议（含 VER/SEQ/LEN、CRC-16/CCITT-FALSE、
  7 条命令、5 种遥测）在 [`acg720_protocol.md`](acg720_protocol.md) 里逐条标注了
  `文件:行号`，并与厂商的 `chassis_control.py` 做了逐字节交叉核对（见
  `test_protocol_v1.py` 的 4 条活体测试）。
* **CPR = 1320**：`encoder_quadrature.v:6` 的注释写明
  "MG520 1:30 为 1320 计数/输出轴一圈，50 ms 测速时 RPM=计数×10/11"。
  从"1320 计数/圈 + 65 mm 轮径"独立推出的 **0.5 m = 3232 计数**，
  正好命中交接包示例里的 `POSITION(1, 25, 3232)` —— 两个来源互相印证。
* **算力平台**：香橙派 5 Plus（RK3588，aarch64）。宿主是 Ubuntu 22.04 的
  Rockchip BSP 内核，**不改宿主系统**，用 `docker/` 在容器里跑 Noble + Jazzy。
  板子上已验证（2026-10-16，`ros:jazzy-ros-base` 容器内）：`platform_check.py`
  无阻塞项、`pytest` 344 passed / 5 skipped（5 项厂商交叉核对按设计跳过 ——
  交接包不在板上）、4A 分段导航 HIL 65/65、运动模式 HIL 59/59、
  **launch 冒烟 32/32（`exit=0`，`motion_cmds=16`，里程计 0.0 → 0.167 m）**。
  冒烟测试第一次在板上跑**暴露了测试自己的一个竞态**（`ros2 topic pub` 默认
  不等订阅端匹配就开发，慢平台上 1.2 s 窗口会被 DDS 发现吃光），
  在 `0a50239` 修掉后板子复跑通过 —— 至此"容器内跑真 `ros2 launch` +
  虚拟底盘 + 收 `/cmd_vel` 真的动起来"这条链路在目标硬件上闭环了。

### ⛔ 仍然挡路的

1. **板上到底烧的是哪个位流。** 交接包含两个 `.fs`，而 `SHA256.txt` **只登记了
   `23_wifi_car_curve_model.fs`**；`24_wifi_car_position_loop.fs` 没登记。
   好在固件在 0x83 遥测里报 `build_id`（当前 `0x20260A02`），
   驱动的启动路径会校验它，不符就**拒绝一切运动命令**（fail-safe，
   不会按错误口径控制真车）。所以这一条不再需要"上车前盲猜"，
   但**第一次上电时应该看一眼驱动的启动日志**。
2. ~~**连续导航被固件的 3.0 s 运行上限挡住。**~~
   **已降级**：连续 `0x02 MOVE` 仍然会被挡，但 `control_mode:=position`
   （一串 `0x05 POSITION` 段）不改固件就能连续移动，见
   [`position_navigation.md`](position_navigation.md)。
   代价是没有圆弧、平均速度约 0.09 m/s。
   想要连续圆弧运动才需要 FPGA 变更，见
   [`fpga_change_request.md`](fpga_change_request.md)。
   ⚠ **新出现的代价**：分段导航一段发出后由 FPGA 走完（最长约 0.73 m /
   ~8 s），主机**只能在段间中止**。所以动态避障必须靠"段间介入"设计，
   不能指望段内 —— 这是接雷达后要专门验的一条。
3. **雷达型号 / 驱动包 / 安装位姿都还没有。** 用户已确认会加 2D 雷达
   （LD19 / RPLIDAR A1 级），但具体型号未定。`/scan` 是 SLAM 与 AMCL 的前提，
   所以**没有它就无法验证①③两条功能**。`robot.launch.py` 因此默认
   `use_laser:=false` 且不绑定任何型号。
4. **有没有外部真值设备。** 动捕？还是自己搭 AprilTag（天花板相机 + 车顶标签）？
   这决定第 4 节走哪条路线，也决定②"零接触"能不能被证明。

### 优先级建议

```
① 底盘台架验证（架空车轮）：方向、急停、/odom 尺度、位流版本号
     用 robot.launch.py nav:=false allow_run_limit:=true
② 定雷达型号 + 装好驱动，确认 /scan 出得来   ← 不解决，Nav2/SLAM 无从验证
③ 轮子落地，按 real_robot_params.md 标定 wheel_separation_scale
④ 决定 FPGA 变更（fpga_change_request.md 的 P0）→ 决定能不能做连续导航
⑤ 再谈①②③三条功能的真机复现（需要第 4 节的真值方案）
```

**注意 ①③ 现在就能做**，不依赖 ②：`robot.launch.py` 只起底盘和 TF，
不碰雷达也不碰 Nav2。所以"往真机方向推进"的下一步并不被阻塞。
