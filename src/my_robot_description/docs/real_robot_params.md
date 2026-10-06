# 实车待测参数清单

本文件是仿真里**所有依赖实车实测数据**的参数的唯一登记处。

- 参数本体：`urdf/my_robot.urdf.xacro` 顶部第 30–90 行的一块 `<xacro:property>`（共 6 组）。
- 改完**不需要** `colcon build`：`install/` 里是指向源码的符号链接。
  改一行 → `Ctrl-C` → 重跑 launch 即可生效（约 10 秒）。
  （例外：**新增**文件——新的 launch / rviz / 地图——必须重新
  `colcon build --symlink-install`，因为符号链接是按文件建的。）
- 现在这些值**全是按 0.4×0.3 底盘估的占位值**，不是实测值。

> 为什么现在用占位值也能往后推进：仿真是自洽的，定性结论（不翘头、雷达读数与解析解吻合、
> SLAM 能建图、Nav2 能规划）与具体数值无关；只有"需要标定的数值"（速度上限、打滑比、
> 代价地图膨胀半径）将来要重调一遍。

---

## 一、几何 / 质量

测量工具：卷尺 + 电子秤。测完直接填"实测值"列。

| xacro property | 占位值 | 含义 / 怎么量 | 影响什么 |
|---|---|---|---|
| `wheel_offset_x` | 0.14 | **半轴距** = 同侧前后轮中心距 ÷ 2。最关键的一个值 | 抗翘头恢复力矩、原地转里程计误差、Nav2 差速模型 |
| `wheel_offset_y` | 0.17 | **半轮距** = 左右轮中心距 ÷ 2 | DiffDrive 的 `wheel_separation` 自动跟随 |
| `wheel_radius` | 0.05 | 轮子半径。装车压过之后的**实际滚动半径**更准：推车走 1 m，看编码器报多少 | DiffDrive 线速度换算，直接决定里程计尺度 |
| `wheel_width` | 0.03 | 轮胎宽度 | 仅碰撞体形状 |
| `wheel_offset_z` | −0.05 | 轮心相对底盘中心的 z | 底盘离地间隙，见第三节自检 |
| `base_length` | 0.4 | 底盘长（含所有突出物） | Nav2 footprint / 代价地图 |
| `base_width` | 0.3 | 底盘宽 | Nav2 footprint / 代价地图 |
| `base_height` | 0.15 | 底盘高 | 见第三节自检 |
| `base_mass` | 5.0 | 整车质量 **减去** 4 个轮子 | 惯量、加速表现 |
| `wheel_mass` | 0.35 | 单个轮子质量 | 同上 |

## 二、摩擦 / 打滑标定

skid-steer 原地旋转时四个轮子必须横向刮擦，编码器积分出来的转角**必然**与车体真实转角不符。
这是实车同样存在的固有特性，所以仿真里**故意保留这个物理现象**，不是 bug。

但**上报的里程计必须标定**（否则 AMCL/Nav2 会以为车转到了它没转到的角度）。标定量就是
`wheel_separation_scale`：把 DiffDrive 插件里的几何轮距换成"有效轮距"
`b_eff = b · scale`。一个系数同时修好两件事：

```
指令侧：  Δv = ω_cmd · b_eff  →  ω_true = ω_cmd     （控制增益对）
里程计侧：ω_odom = Δv / b_eff = ω_cmd = ω_true      （里程计尺度对）
```

| xacro property | 当前值 | 怎么标定 | 影响什么 |
|---|---|---|---|
| `wheel_mu_long` | 1.0 | 沿滚动方向的摩擦系数，一般保持 1.0 | 牵引力 |
| `wheel_mu_lat` | 0.5 | 垂直滚动方向的摩擦。决定**物理上打滑多少** | 真实转角 → 所以也决定 `scale` 该填多少 |
| `wheel_separation_scale` | **1.36** | 见下 | 控制增益 + 里程计尺度，**Nav2 定位精度的第一决定因素** |

### `wheel_separation_scale` 的标定方法

1. 让实车**原地**以固定角速度 ω 转固定时间 T（比如 0.6 rad/s 转 15 s）。
2. 同时记下两件事：
   * **编码器/里程计积分出来的转角** θ_odom（`ros2 topic echo /odom` 的 yaw，注意
     要累加未缠绕的增量，不能取"末减初"，±180° 有歧义）；
   * **车实际转过的转角** θ_true（地上贴胶带 / 车顶画一条线 / 外部测量）。
3. `scale = θ_odom / θ_true`，填进 `wheel_separation_scale`。
   标定正确后 `θ_odom ≈ θ_true`，且"命令转一圈实际就转一圈"。

⚠ 转弯半径不同侧滑量不同（原地转刮擦最厉害）。单一标量只能折中，
**优先照顾原地转**，因为室内导航（穿门、对准）以原地转为主。

### 当前仿真的实测数据

| 工况 | `odom/真值` 标定前 | 标定后 |
|---|---|---|
| 原地转 ω=+0.6 rad/s | 1.355 | 1.040 |
| 原地转 ω=−0.6 rad/s | 1.355 | 1.000 |
| 原地转 ω=+1.2 rad/s | 1.364 | 1.003 |
| 原地转 ω=+1.5 rad/s | 1.361 | — |
| 弧线 v=0.3 ω=+0.5（r=0.6 m） | 1.436 | 1.091 |

标定前"命令转 1 圈实际只转 0.73 圈"，标定后对上了。完整推导与测量脚本见 `nav2.md`。

> 想让物理上的打滑**变小**（更接近理想差速车）：调小 `wheel_mu_lat`。
> 但真实 skid-steer 就是会打滑，把它调没等于自欺欺人 —— 正确做法是保留物理、
> 标定上报口径。

## 三、速度 / 加速度上限

| xacro property | 占位值 | 怎么量 | 影响什么 |
|---|---|---|---|
| `max_linear_vel` | 0.5 | 实车最大安全直线速度 (m/s) | DiffDrive 限幅 + 将来 Nav2 速度上限 |
| `max_angular_vel` | 1.5 | 最大角速度 (rad/s) | 同上 |
| `max_linear_acc` | 0.5 | 加速度 (m/s²) | 启停平顺性 |
| `max_angular_acc` | 2.0 | 角加速度 (rad/s²) | 同上 |

## 四、激光雷达

以下值**从未和实车雷达核对过**。

| xacro property | 占位值 | 说明 |
|---|---|---|
| `lidar_mount_x/y/z` | 0.0 / 0.0 / 0.12 | 相对 `base_link` 的安装位置。按实车型号手册 + 卷尺量 |
| `lidar_samples` | 360 | 水平线数 |
| `lidar_min_angle` / `lidar_max_angle` | −3.14159 / 3.14159 | 水平视场角 |
| `lidar_range_min` / `lidar_range_max` | 0.1 / 12.0 | 量程。**量程填小了会把真实存在的墙当成超出量程丢掉** |
| `lidar_update_rate` | 10 | Hz |
| `lidar_noise_stddev` | 0.01 | 测距噪声标准差 (m)。实车这项远大于 1 cm |

---

## 五、连带修改对照表

改 URDF 参数时，下面这些地方**必须一起改**，否则会出现"地图建歪 / 导航贴墙"却查不出原因的情况。

| 改了 URDF 里的 | 还要改 | 状态 |
|---|---|---|
| `base_length` / `base_width` / `wheel_offset_y` / `wheel_width` | Nav2 的 `footprint`，**局部和全局代价地图各一份**（`config/nav2_params.yaml`） | ✅ 已建 |
| `max_linear_vel` / `max_angular_vel` | Nav2 MPPI 的 `vx_max` / `wz_max`、`velocity_smoother` 的 `max_velocity`/`min_velocity`、`behavior_server.max_rotational_vel` | ✅ 已建 |
| `max_linear_acc` / `max_angular_acc` | Nav2 MPPI 的 `ax_max`/`ax_min`/`az_max`、`velocity_smoother` 的 `max_accel`/`max_decel`、`behavior_server.rotational_acc_lim` | ✅ 已建 |
| `lidar_range_max` | `slam_toolbox` 的 `max_laser_range`（`config/slam_toolbox.yaml`） | ✅ 已建 |
| `lidar_range_min` / `lidar_range_max` | AMCL 的 `laser_min_range` / `laser_max_range` | ✅ 已建 |
| `lidar_range_max` / `lidar_mount_z` | Nav2 costmap 的 `obstacle_max_range` / `raytrace_max_range` | ✅ 已建 |
| `lidar_update_rate` / `lidar_noise_stddev` | `slam_toolbox` 的 `minimum_time_interval`、`correlation` 搜索窗 | ✅ 已建 |
| `wheel_mu_lat` | 改的是**物理**打滑量，因此要重新标定 `wheel_separation_scale` | — |
| `wheel_separation_scale` | 无需改配置，但它直接决定 AMCL 的定位质量 | ✅ |
| `wheel_offset_*` / `wheel_radius` / 质量 | DiffDrive 的 `wheel_separation` / `wheel_radius` 由 xacro 自动推导，**无需另改** | ✅ |
| `base_mass` / 质量分布 | 影响加速表现和打滑量，间接影响 `wheel_separation_scale` | — |

⚠ **`base_length`/`base_width` 改大时别忘了 `<inflation_radius>`**：它应该约等于
"外接圆半径 + 5~8 cm 余量"，本工程取 0.35（外接圆 0.273）。膨胀半径太大，
0.9 m 宽的门会变成高代价区甚至过不去。

## 六、改完后的自检

### 1. 几何一致性（改了 `base_height` / `wheel_offset_z` / `wheel_radius` 后必查）

底盘底面必须高于轮子底面，否则车会卡在地上：

```
底盘底面 z = -base_height / 2          = -0.075
轮子底面 z = wheel_offset_z - wheel_radius = -0.050 - 0.050 = -0.100
要求：-base_height/2 > wheel_offset_z - wheel_radius
```

当前离地间隙 = **2.5 cm**，成立。改动这三个值后按上式复核。

另外 `wheel_offset_x` 必须 > 0（前后轴要分开），否则抗翘头恢复力矩退回 0.3 N·m，车会抬头。

### 2. 展开后语义不变（只调整写法时用）

```bash
xacro src/my_robot_description/urdf/my_robot.urdf.xacro | head -60
```

### 3. 运行时抽查

```bash
# 雷达安装位置：应等于 lidar_mount_z
ros2 run tf2_ros tf2_echo odom laser_frame

# 雷达参数是否生效：angle_increment = (max-min)/(samples-1)
ros2 topic echo /scan --once --full-length | head -20

# 里程计线速度尺度：命令走 1.0 m，看 odom 累计 x
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}}" &
ros2 topic echo /odom --field pose.pose.position

# ★ 里程计角速度尺度（= wheel_separation_scale 的前提，见第二节）
#   原地转，比对 odom 累计转角 与 车实际转角。标定正确时两者相等。
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{angular: {z: 0.6}}" &
ros2 topic echo /odom --field pose.pose.orientation
```

### 4. 改完 Nav2 参数后（`config/nav2_params.yaml`）

新增/修改配置文件**不需要** `colcon build`（除非是新增文件），
但**新增**文件必须重新 `colcon build --symlink-install`，否则 `install/` 里没有链接。

```bash
# 打通全链路 + 打一个目标点，观察终点误差（真值口径）
ros2 launch my_robot_description nav.launch.py
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.9}, orientation: {w: 1.0}}}}"

# 看代价地图有没有把门口堵死（膨胀半径太大的典型症状）
ros2 topic echo /global_costmap/costmap --once | head -5
```

如果目标点总是"明明到了却差几十厘米"，先按第二节复核 `wheel_separation_scale`，
再看 AMCL 的观测模型参数（`nav2.md` 里"AMCL 观测模型照抄上游会收不紧"一节）。
