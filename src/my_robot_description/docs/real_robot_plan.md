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
| 驱动层（底盘 / 雷达 / TF） | ❌ 完全没有，现在是 Gazebo 插件 | **最大的一块** |
| 参数标定 | ⚠️ 清单已备好，值全是占位 | 中等，但顺序不能错 |
| **验证手段** | ❌ **现在的方法真机上失效** | 被低估的一块 |
| 安全措施 | ❌ 仿真里不存在这个概念 | 必须补齐才能上车 |

三条功能里，**① 起点到终点** 和 **③ 路径点** 搬到真机主要是标定问题；
**② 障碍物避让** 最麻烦，因为**"零接触"这个结论在真机上没法用现在的工具证明**（见第 4 节）。

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

### 2.2 顺带要修的几处

读 `launch/nav.launch.py` 发现的、`sim:=false` 时会碍事的地方（**仅读代码得出，未实跑验证**）：

| 行 | 问题 | 建议改法 |
|---|---|---|
| 194、224、262 | `use_sim_time` **三处硬编码 `'true'`** | 提成 launch 参数，跟随 `sim` |
| 162-163 | 无条件检查 `worlds/<world>` 文件存在 | `sim_on` 时才检查 |
| 138、282 | `world` 参数在非仿真下无意义 | 保留但跳过校验 |

### 2.3 还需要一个新的 bringup launch

现在是三段式，真机上车需要一层把它们串起来：

```
现在：  gazebo_sim.launch.py   （仿真 + 桥）
        slam.launch.py         （单独建图）
        nav.launch.py          （Nav2，sim:= 可关仿真）

要加：  robot.launch.py        （新增）
          ├── 底盘驱动节点
          ├── 雷达驱动节点
          ├── robot_state_publisher（URDF 去掉 gz 插件）
          └── nav.launch.py  sim:=false
```

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

真机落地时预计要动的文件：

| 文件 | 动作 |
|---|---|
| `urdf/my_robot.urdf.xacro` | **改**：删 gz 插件与 sensor；按实测填 6 组 property |
| `launch/robot.launch.py` | **新增**：驱动 + 雷达 + `robot_state_publisher` + `nav.launch.py sim:=false` |
| `launch/nav.launch.py` | **改**：`use_sim_time` 提成参数（194/224/262 行）；world 校验加条件（162 行） |
| `config/nav2_params.yaml` | **改**：`footprint`、`inflation_radius`、速度/加速度上限、AMCL 雷达参数 |
| `config/slam_toolbox.yaml` | **改**：`max_laser_range`、`minimum_time_interval` |
| `maps/*` | **替换**：真场地重新建图 |
| `scripts/mapeval.py` | **改或新增**：现在依赖 world SDF，真机需要一个不依赖真值的替代口径 |
| `scripts/obs_test.py` | **改或新增**：同上，真机版要接外部真值源 |
| `docs/real_robot_params.md` | **填**：把占位值换成实测值 |
| 新增 `scripts/` | 可能：外部真值（AprilTag / 动捕）的接入脚本 |

**不需要改**：`cost_slice.py`、`nav_goals.py`、`routes/*`。
`CMakeLists.txt` 的 install 列表也不用改——它已经是 `install(DIRECTORY ... docs scripts ...)`，
覆盖了 `docs/` 和 `scripts/` 下的所有文件。

⚠️ **但新增文件后必须重新 `colcon build --symlink-install`。** `install/` 里是**按文件**建的
符号链接（`real_robot_plan.md -> .../src/.../docs/real_robot_plan.md`），不是整目录链接，
所以 CMake 的 `DIRECTORY` 规则虽然涵盖了它，也要跑一次 build 才会生成链接。
（本文档本身就撞过这个：写完 `ls install/.../docs/` 里没有，build 一次才出现。
`real_robot_params.md` 开头也记了同一条。）

---

## 8. 阻塞项：先确认硬件

**下面这些没确认之前，第 2 节的驱动层没法动手**——因为 gz 插件换成什么节点，完全取决于硬件：

1. **底盘**：什么型号？有没有现成 ROS 驱动？还是只有电机 + 编码器要自己写？
2. **编码器接口**：串口 / CAN / USB？协议文档有没有？
3. **雷达**：什么型号？（RPLIDAR / Livox / 其他）驱动包是哪个？量程和视场角是多少？
4. **算力平台**：车上是什么？（Jetson / 树莓派 / NUC）跑得动 Nav2 吗？
5. **有没有外部真值设备**：动捕？还是需要自己搭 AprilTag（决定第 4 节走哪条路线）？

把这五条确认了，才能给出「具体改哪个文件、加哪个节点、每步怎么验」的落地版本。
