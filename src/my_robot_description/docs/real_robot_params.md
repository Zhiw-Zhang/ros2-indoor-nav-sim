# 实车待测参数清单

本文件是仿真里**所有依赖实车实测数据**的参数的唯一登记处。

- 参数本体：`urdf/my_robot.urdf.xacro` 顶部第 30–70 行的一块 `<xacro:property>`。
- 改完**不需要** `colcon build`：`install/` 里是指向源码的符号链接。
  改一行 → `Ctrl-C` → 重跑 launch 即可生效（约 10 秒）。
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
这是实车同样存在的固有特性，所以仿真里**故意保留**，不是 bug。

| xacro property | 占位值 | 怎么标定 | 影响什么 |
|---|---|---|---|
| `wheel_mu_long` | 1.0 | 沿滚动方向的摩擦系数，一般保持 1.0 | 牵引力 |
| `wheel_mu_lat` | 0.5 | 垂直滚动方向的摩擦。**唯一的标定旋钮** | 打滑程度 → `odom/true` 转角比 |

标定方法：让实车**原地转一整圈**，记录 `命令角速度 × 时间` 与里程计累计转角的比值。

- 当前仿真的比值：`odom/true ≈ 1.38`（命令 0.8 rad/s × 8 s，真实转了 287.9°）。
- 想让比值 → 1.0：调小 `wheel_mu_lat`；想更打滑：调大。

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
| `base_length` / `base_width` | Nav2 的 `footprint`（`config/nav2_params.yaml`） | 待建 |
| `max_linear_vel` / `max_angular_vel` | Nav2 的 `max_vel_x` / `max_vel_theta`、`controller` 速度上限 | 待建 |
| `max_linear_acc` / `max_angular_acc` | Nav2 的 `acc_lim_x` / `acc_lim_theta` | 待建 |
| `lidar_range_max` | `slam_toolbox` 的 `max_laser_range` | 待建 |
| `lidar_range_max` / `lidar_mount_z` | Nav2 costmap 的 `obstacle_max_range` / `raytrace_max_range` | 待建 |
| `lidar_update_rate` / `lidar_noise_stddev` | `slam_toolbox` 的 `minimum_time_interval`、`correlation` 搜索窗 | 待建 |
| `wheel_mu_lat` | 无需改配置，只影响里程计质量 | — |
| `wheel_offset_*` / `wheel_radius` / 质量 | DiffDrive 参数由 xacro 自动推导，**无需另改** | ✅ |

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

# 里程计尺度：命令走 1.0 m，看 odom 累计 x
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}}" &
ros2 topic echo /odom --field pose.pose.position
```
