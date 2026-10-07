# sim_nav_ws

室内自主导航仿真：一台四轮 skid-steer 小车，用 **SLAM 建图 + Nav2 自主导航**，
在 Gazebo 里跑通「起点到终点自主移动 / 障碍物避让 / 路径点导航」三条功能。

ROS 2 **Jazzy** + Gazebo Sim **8.15**。

---

## 验收结论

| # | 需求 | 结论 | 一句话 |
|---|---|---|---|
| ① | 起点到终点自主移动 | ✅ | 空场地 6/6；带障碍物场地绕行 2.91× 到达，终点误差 6.6~20.4 cm |
| ② | 障碍物避让 | ✅（静态障碍物） | 5 个障碍物（含 6 cm 细杆）全部绕开，三次测试**零接触**，不是靠兜底刹车 |
| ③ | 路径点导航 | ✅ | 一次调用 5/5；路点不可达时报准确错误码并跳过 |

**完整测试方法、数据和没通过的部分见 [`docs/acceptance.md`](src/my_robot_description/docs/acceptance.md)。**

两个已知问题（都写在上面那份文档里，不藏着）：

1. **窄通道会卡死**：`obs_roomB_slant` 东端 0.726 m 的缝可复现卡住。
   根因是 `inflation_radius: 0.35` 需要 0.70 m，只剩 2.6 cm 余量。
2. **车前突然出现的障碍物会卡死**：安全刹停有效，但之后恢复行为陷入死循环。

---

## 环境要求

* Ubuntu 24.04 + ROS 2 Jazzy
* Gazebo Sim 8（`ros_gz`）
* WSL2 + WSLg **可以直接跑**——`launch/gazebo_sim.launch.py` 会自己检测 WSL
  并强制软件渲染（`LIBGL_ALWAYS_SOFTWARE=1` + `GALLIUM_DRIVER=llvmpipe`），
  同时保留 WSLg 的 `DISPLAY=:0`。**不需要手动 export 这些变量。**

依赖包（一条命令装齐）：

```bash
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-nav2-map-server ros-jazzy-xacro ros-jazzy-rviz2
```

> **也可以走 `rosdep`**，但本机的 rosdep **从未初始化**——
> `/etc/ros/rosdep/sources.list.d/` 不存在，直接跑会报
> `your rosdep installation has not been initialized yet`。要用它得先：
>
> ```bash
> sudo rosdep init      # 需要 sudo
> rosdep update         # 需要网络
> rosdep install --from-paths src --ignore-src -r -y
> ```
>
> `package.xml` 里的依赖已经登记齐了（1 条 buildtool + 17 条 `exec_depend` + 2 条
> `test_depend`，含 `nav2_bringup`、`slam_toolbox`、`ros_gz_sim`、`ros_gz_bridge` 等），
> 所以上面三条跑通后就能自动装齐。

---

## 快速开始

```bash
cd ~/sim_nav_ws
colcon build --symlink-install
source install/setup.bash
```

### 1. 导航（最常用）

```bash
ros2 launch my_robot_description nav.launch.py
```

起 Gazebo + map_server + AMCL + Nav2 + RViz，等约 15 s 车 spawn 出来，
在 RViz 里用工具栏的 **Nav2 Goal** 点一个目标（点一下定位置，再拖一下定朝向）。

### 2. 建图

```bash
ros2 launch my_robot_description slam.launch.py
# 另开终端遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 建完保存
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### 常用变体

```bash
# 只起 Nav2，不起 Gazebo（Gazebo 已在别处跑着）
ros2 launch my_robot_description nav.launch.py sim:=false

# 不开 RViz（省内存 / 无头验证）
ros2 launch my_robot_description nav.launch.py rviz:=false

# 边建图边导航
ros2 launch my_robot_description nav.launch.py slam:=true

# 换 world / 换地图 / 换参数
ros2 launch my_robot_description nav.launch.py world:=empty.sdf
ros2 launch my_robot_description nav.launch.py map:=/abs/path/other.yaml
```

---

## 仓库结构

```
sim_nav_ws/
└── src/my_robot_description/
    ├── urdf/my_robot.urdf.xacro   机器人模型 + gz 插件（差速驱动、雷达）
    │                              ★ 顶部 30~90 行是 6 组待标定的 property
    ├── launch/
    │   ├── gazebo_sim.launch.py   仿真 + ros_gz 桥（含 WSL 渲染适配）
    │   ├── nav.launch.py          Nav2（sim:= 可关仿真）
    │   └── slam.launch.py         建图
    ├── config/
    │   ├── nav2_params.yaml       Nav2 全部参数（改过的地方标了 ★）
    │   ├── slam_toolbox.yaml      SLAM 参数（两处必改的写了注释）
    │   └── *.rviz                 三个 RViz 配置
    ├── worlds/                    三个场景（见下表）
    ├── maps/                      三张地图，与 worlds 一一对应
    ├── routes/                    路点文件（供 nav_goals.py 使用）
    ├── scripts/                   五个验收工具（见 docs/tools.md）
    └── docs/                      ★ 文档入口：docs/README.md
```

### 三个场景

| world | 地图 | 说明 |
|---|---|---|
| `rooms.sdf` | `rooms.yaml` | 两间房 + 走廊，无障碍物。基线场景 |
| `rooms.sdf` | `rooms_manual.yaml` | 同一场景，**手动遥控**建图，质量略好 |
| `rooms_obstacles.sdf` | `rooms_obstacles.yaml` | rooms **+ 5 个静态障碍物**，门AB 被封死 |

第三张图用自动覆盖路线建（17 个路点，303 s），5 个障碍物全部进图。
用它要同时换 world 和 map：

```bash
ros2 launch my_robot_description nav.launch.py world:=rooms_obstacles.sdf \
  map:=$(ros2 pkg prefix my_robot_description)/share/my_robot_description/maps/rooms_obstacles.yaml
```

---

## 文档

**从 [`docs/README.md`](src/my_robot_description/docs/README.md) 进**——那是文档地图，
含推荐阅读顺序和"同一个问题该信哪份文档"的权威出处对照。

| 文档 | 讲什么 |
|---|---|
| [acceptance.md](src/my_robot_description/docs/acceptance.md) | 三条需求的验收判决（含没通过的） |
| [nav2.md](src/my_robot_description/docs/nav2.md) | Nav2 调参细节、定位精度实测、踩过的坑 |
| [slam.md](src/my_robot_description/docs/slam.md) | `slam_toolbox` 建图与两处必改配置 |
| [tools.md](src/my_robot_description/docs/tools.md) | 五个验收脚本 |
| [real_robot_plan.md](src/my_robot_description/docs/real_robot_plan.md) | **实车迁移计划** |
| [real_robot_params.md](src/my_robot_description/docs/real_robot_params.md) | **实车参数标定手册** |

---

## 真机状态

**真车（ACG720）上还没有跑过，目前没有任何真机实测数据。**

搬到真车要做的事（验证手段、安全、分阶段推进）整理在
[`real_robot_plan.md`](src/my_robot_description/docs/real_robot_plan.md)。

### 底盘驱动层：已实现，且**没有硬件也能验证**

ACG720 底盘的串口驱动层已经完成（协议、驱动节点、虚拟底盘、真机 bringup）：

```bash
# 没有硬件：起虚拟底盘，它会打印 /dev/pts/N
ros2 run my_robot_description acg720_simulator --no-driver

# 把真机 bringup 指到那个 PTY —— /odom、TF、超时停车全走真实代码路径
ros2 launch my_robot_description robot.launch.py port:=/dev/pts/N nav:=false
```

细节见 [`acg720_protocol.md`](src/my_robot_description/docs/acg720_protocol.md)。
验证规模：**54 例纯函数单测 + 30 项 HIL 集成测试 + 20 项 launch 冒烟测试**，
含 skid-steer 标定闭环（`θ_odom/θ_true` 从 1.361 收到 1.005）。

### 还差的两块

1. **雷达**——已确认会加 2D 雷达（LD19 / RPLIDAR A1 级），但型号与安装位姿未定。
   `/scan` 是 SLAM 与 AMCL 的前提，所以**没有它就无法验证三条功能**。
   `robot.launch.py` 因此默认 `use_laser:=false`，且不绑定任何雷达型号。
2. **验证手段会失效**——`mapeval.py` 和 `obs_test.py --static` 都依赖仿真真值位姿，
   真机上没有对应物。②"零接触"这个结论在真机上**目前无法证明**。

### 接真车前必须解决的两条

* ⚠️ **串口 payload 字节布局没有权威来源。** 交接包只给了帧头 `A5 5A` 和命令字
  （`0x05`/`0x06`/`0x84`），没给字段顺序、字节序和校验方式。本工程自行定义了一套，
  集中在 `src/acg720_protocol.py` **一个文件**里，必须拿 FPGA 源码核对后才能接真车。
  布局不对的后果是底盘按错误速度跑，而日志上一切正常。
* ⚠️ **两套 URDF 的物理参数都未经实车复核。** 仿真用的
  [`my_robot.urdf.xacro`](src/my_robot_description/urdf/my_robot.urdf.xacro) 是按
  0.4×0.3 底盘估的占位值（含 `wheel_separation_scale = 1.36`、
  `inflation_radius = 0.35`）；真车用的
  [`acg720_real.urdf.xacro`](src/my_robot_description/urdf/acg720_real.urdf.xacro)
  按交接包几何填（轮径 65 mm、轮距 200 mm、轴距 185 mm），但同样**未实测**。
  清单与测量方法见 [`real_robot_params.md`](src/my_robot_description/docs/real_robot_params.md)。

> ⚠️ 真机上的 `wheel_separation_scale` **必须重量，不要抄仿真的 1.36**——
> 那是 ODE 物理引擎 + `wheel_mu_lat=0.5` 的产物，与真车轮胎/地面/负载无关。
> 真机初始值填 `1.0`，按标定流程填实测值。
