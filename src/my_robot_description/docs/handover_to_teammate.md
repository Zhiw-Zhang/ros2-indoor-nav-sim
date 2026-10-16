# 交接单：真车部分交给谁、按什么顺序做、哪里不能碰

> **这份文档的读者是接手硬件的人**（底盘/FPGA/雷达/标定），不需要读我们之前的
> 排查过程。要细节时按每节末尾给的链接进对应文档。
> 软件侧的人看 [`docs/README.md`](README.md)；想知道"为什么这么设计"看
> [`acg720_protocol.md`](acg720_protocol.md)。

---

## 0. 一句话现状

**软件栈已经在"虚拟底盘"上验证完毕，并在香橙派上跑通；但真电机、真编码器、
真雷达一次都没有被命令过。**

虚拟底盘不是"随便模拟一下"：它是按 FPGA 的 RTL 逐行复刻的固件模型
（`src/acg720_firmware.py`），走的是**真协议的字节**，包含 3.0 s 连续运动上限、
300 ms 命令超时、ACK 单槽、FAULT 自锁这些真实行为。所以协议层、驱动层、
分段导航、"车会不会被锁死"这些问题都已经在模型上验过了，**不需要你在真车上
再发现一遍**。

分工一句话：**你负责"硬件能动、数据能出来"，软件侧负责"数据能变成车的位置和
动作"**。交界处只有几个固定的名字（见第 3 节）。

---

## 1. 先跑这几条命令（不需要任何硬件）

仓库已经在板子上（`~/sim_nav_ws`）。**动手前先和软件侧核对一次版本**，
免得你在调的代码和我们对不上：

```bash
git -C ~/sim_nav_ws log -1 --oneline     # 把这个 commit 发给软件侧确认
```

目的只有一个：**先知道"正常长什么样"**。之后真车上出问题，才能分清是硬件还是
软件。四条都在板子上、容器里跑：

```bash
cd ~/sim_nav_ws

# ① 构建镜像（只第一次）+ 构建工作空间
./docker/run.sh --build                        # 镜像，只做一次（慢）
./docker/run.sh colcon build --symlink-install  # 工作空间
#    ⚠ 新增/改名文件后**必须重跑** colcon build：install/ 里是按**文件**
#      建的符号链接，不重跑就会出现"脚本明明在，ros2 run 找不到"。

# ② 平台自检：内核/容器/ROS/依赖/串口与 PTY/启动参数/CPU/网络
./docker/run.sh ros2 run my_robot_description platform_check.py
#    判据：结尾「没有硬阻塞 ✓」

# ③ 真 ros2 launch + 虚拟底盘 + /cmd_vel 真的推动里程计
./docker/run.sh python3 src/my_robot_description/test/test_bringup_smoke.py
#    判据：结尾「全部通过 ✓（32 项）」，exit=0

# ④ 分段导航 HIL（65 项）
./docker/run.sh python3 src/my_robot_description/test/test_position_nav_hil.py
#    判据：全绿

# 全部单测（板上应当是 344 passed / 5 skipped；5 项跳过是设计好的，
# 它们要交接包的厂商文件，那包不在这台机器上）
./docker/run.sh python3 -m pytest src/my_robot_description/test
```

含硬件体检的三个脚本也可以先"空跑"确认脚本本身没问题：

```bash
./docker/run.sh ros2 run my_robot_description scan_check.py --selftest
./docker/run.sh ros2 run my_robot_description acg720_simulator.py --dry-run
```

> 容器怎么起的、为什么要 `--network host --ipc host`、真串口怎么进去，
> 都在 [`docker/README.md`](../../../docker/README.md)。**不要**改宿主系统，
> 也不要把 `docker/run.sh` 换成手敲的 `docker run`。

想手工看虚拟底盘长什么样（不跑测试）：

```bash
./docker/run.sh python3 src/my_robot_description/src/acg720_simulator.py
# 打印 PTY slave 路径，例如 /dev/pts/1
# 另一个终端把驱动指过去：
./docker/run.sh ros2 run my_robot_description acg720_driver.py \
  --ros-args -p port:=/dev/pts/1 -p control_mode:=position
```

---

## 2. 分工边界

| | 你（硬件侧） | 软件侧 |
|---|---|---|
| FPGA / RTL / 位流 | ✅ 烧写、版本管理、P0 变更 | 只提需求（[`fpga_change_request.md`](fpga_change_request.md)） |
| 电机 / 编码器 / 接线 / 结构 | ✅ | — |
| 雷达选型、安装、供电、驱动包 | ✅ 装好、能用 | 从 `/scan` 往下的一切 |
| 标定测量（卷尺、贴胶带、推车） | ✅ 动手测 | 给方法、判读数据、填参数 |
| 协议帧、驱动节点、分段导航 | — | ✅ `src/acg720_*` |
| Nav2 / SLAM / AMCL 配置 | 提供真值（尺寸、雷达参数） | ✅ `config/` |
| **交界处的名字与数值** | **改名/改数值必须两边同时改** | 同左 |

⚠ 唯一容易出事的协作点：**改名**。话题名、frame 名、设备名、URDF 参数名
都被配置文件引用着，单边改动会出现"什么都对但就是不动"。

---

## 3. 冻结的接口契约（这些名字不要单方面改）

| 项 | 冻结值 | 出现在哪 |
|---|---|---|
| 扫描话题 | `/scan` | `config/nav2_params_real.yaml`、`config/slam_toolbox.yaml` |
| 雷达 frame | `laser_frame` | URDF、RViz 配置、文档 |
| 车体 frame | `base_link` | URDF、`nav2_params_real.yaml` 的 `robot_base_frame` |
| 里程计话题 / frame | `/odom`、`odom` | 驱动发布；Nav2 的 `odom_topic` |
| 目标话题（分段导航） | `/acg720/goal`（`PoseStamped`） | [`position_navigation.md`](position_navigation.md) |
| 分段状态话题 | `/acg720/segment_status`（**latched**） | 同上 |
| 诊断话题 | `/acg720/diagnostics` | 驱动 |
| 底盘串口设备名 | `/dev/acg720`（udev 固定，别写 `ttyUSB0`） | [`docker/README.md`](../../../docker/README.md) |
| 波特率 | **115200 8N1** | FPGA 侧 50 MHz 整数分频 |
| 接线 | 转接板 TXD→FPGA `E17`、RXD→`C20`、**GND 共地**、3.3 V | [`docker/README.md`](../../../docker/README.md) |
| IMU 那一路 UART | `imu_rxd = D16`（独立于主机链路） | `acg720_60k.cst` |
| 期望位流版本 | `build_id = 0x20260A02` | 驱动启动时**校验**，不符就拒绝一切运动命令 |
| 雷达安装位姿参数 | `lidar_mount_x/y/z` | `urdf/acg720_real.urdf.xacro` |
| 轮距标定系数 | `wheel_separation_scale` | 驱动参数 / launch |

判据脚本（我们已写好，见第 8 节）会逐条检查这张表。

---

## 4. 红线：这些操作会把底盘**锁死**或弄危险

`FAULT` 是**自锁**的：置位之后任何命令都不理，**唯一出路是断电，或者
「先松开 S0，再按 S4」**。所以下面每一条都不是"注意事项"，是硬规则。

| 不要做 | 后果 | 依据 |
|---|---|---|
| 发 CRC 算错的帧 | 判非法 → **永久锁死** | `chassis_command_rx.v:113,191` |
| 发 `LEN > 8` 的帧 | 同上。**`INTERFACE.md` 写的 48 是错的** | `chassis_command_rx.v:208` |
| 发取值越界的命令（如 POSITION 的 `max_rpm` 不在 8~40） | 同上 | `chassis_command_rx.v:97-112` |
| 两条命令间隔 < 15 ms | ACK 单槽挤车 → `tx_overflow` → 锁死。主机侧留 **50 ms** | `chassis_telemetry_imu_tx.v:236-238` |
| 在**运动期间**发 HEARTBEAT | 车走走停停，状态机还显示 RUNNING（运动期间要保持的是**重发 MOVE**） | `top.v:238-281` |
| 用"周期 STOP + 重新 MOVE"绕 3.0 s 上限 | **交接包明令禁止**；车每 3 s 急停一次 | `INTERFACE.md` |
| 在 `position` 模式里发 MOVE | 段结束那一拍残留手动目标接管，车会冲出去 | `top.v:152` |
| 位置段**运行中**再发 POSITION | 静默丢弃（`start_valid && !busy`），既不停也不重启 | `chassis_position_controller.v:177` |
| ARMED 状态下 300 ms 不发任何有效命令 | 锁 `FAULT_TIMEOUT`。所以"保持 ARM"必须持续喂狗（10 Hz） | `chassis_safety_fsm.v:104` |
| 主机重启 / 串口重连后不重新对齐 SEQ | 新序号被判"落后"**静默丢弃** → 车不动、日志无错 | `chassis_command_rx.v:93-95` |
| 连续 `0x02 MOVE` 超过 3.0 s | 锁 `FAULT_RUN_LIMIT`（重发命令**不清零**计时） | `chassis_safety_fsm.v:57-58,72-77` |

另外两条物理上的：

* **轮子必须离地**才能做台架测试。3.0 s 上限和 300 ms 超时在台架上会真实生效。
* **电平 3.3 V**、**必须共地**。5 V 有损坏 FPGA IO 的风险；不共地会随机出错
  （比干脆不通更难查）。

怎么复位：**断电，或者按住 S4 复位；若刚按过急停，要先把 S0 松开**。

细节与 RTL 行号： [`acg720_protocol.md`](acg720_protocol.md) 第 0 节。

---

## 5. 你的任务顺序

每一步都给"前置、命令、判据、下一步"。**按顺序做，不要跳**。

### 步骤 1：接线体检（只读，零风险）

```bash
# 宿主上先确认设备名（别写死 ttyUSB0，用 udev 固定成 /dev/acg720）
ls -l /dev/acg720

# 容器里跑体检：默认**一个字节都不发**，只被动读遥测
ACG720_DEVICE=/dev/acg720 ./docker/run.sh \
  ros2 run my_robot_description real_bringup_check.py --port /dev/acg720

# 想连下行链路一起验（会 ARM 再 DISARM，**轮子不会转**）
ACG720_DEVICE=/dev/acg720 ./docker/run.sh \
  ros2 run my_robot_description real_bringup_check.py --port /dev/acg720 --arm
```

> **还没配 udev 规则？** 那就先按实际路径来（把 `ttyUSB0` 换成 `ls` 看到的那个）：
> `ACG720_DEVICE=/dev/ttyUSB0 ./docker/run.sh ros2 run my_robot_description
> real_bringup_check.py --port /dev/ttyUSB0`。
> ⚠ 两点：**设备节点是 `docker run` 那一刻拿到的，拔插之后必须重启容器**；
> 插拔顺序变了 `ttyUSB0` 可能变成 `ttyUSB1`，所以正式用之前请按
> [`docker/README.md`](../../../docker/README.md) 写一条 udev 规则固定名字。
> 雷达也会是**另一个** `/dev/ttyUSB*`，容器里得同时挂两个设备 ——
> 这件事由软件侧改 `docker/run.sh`，你把接口形态告诉我们就行（见任务步骤 5）。

* **判据**：结尾「接线这一段通了 ✓」，且 `build_id = 0x20260A02`、`fault=NONE`。
* 它会逐条告诉你是"收不到"、"收到一堆垃圾字节"、"有发布没数据"还是"锁在 FAULT"，
  并给出对应的排查项。**不要靠猜**。
* FPGA 从复位起就以 20 Hz 广播遥测（`chassis_telemetry_imu_tx.v:230`），
  所以只读就能判断链路是否通。

### 步骤 2：确认位流

同上一步的 `build_id`。若报**位流版本不符**，先别往下走 —— 交接包里有
`23_wifi_car_curve_model.fs` 和 `24_wifi_car_position_loop.fs` 两个位流，
而 `SHA256.txt` **只登记了前者**。分段导航需要 position_loop 那版。
驱动遇到版本不符会**拒绝一切运动命令**（这是设计好的保护，不是故障）。

> 请顺手把**实际烧的那个 `.fs` 的 SHA256** 记下来回传（见第 7 节）。

### 步骤 3：台架验证（轮子架空）

```bash
./docker/run.sh ros2 launch my_robot_description robot.launch.py \
  nav:=false control_mode:=position
```

另一个终端发一个 0.3 m 的目标（`position` 模式**不需要** `allow_run_limit`，
那个参数只针对连续 MOVE）：

```bash
./docker/run.sh ros2 topic pub --once /acg720/goal \
  geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 0.3, y: 0.0}}}'
```

* **判据**：四个轮子朝**同一方向**转（对应"前进"），里程计 x 增加约 0.3，
  `/acg720/diagnostics` 里 `speed_rpm` 的 A/B/C/D **四个都是正数**、
  `fault=NONE`、`crc_errors=0`，`/acg720/segment_status` 出现 `ARRIVED`。
* 方向反了/歪了 → 记下来回传，这是接线或轮序的问题，**不要**靠软件改正。
* ⚠ 两个坑：目标必须在驱动起来**之后**发（订阅匹配前发的会丢）；只想"到点即可"
  时**不要**写 `orientation`（ROS 默认四元数是"朝向 +x"，会变成"到点还要转回 +x"）。
* **顺手把急停也验了**（现在轮子离地，是最安全的时机）：按住 S0，`/acg720/diagnostics`
  应当立刻变成 `fault=ESTOP`、`state=FAULT`，电机使能断开、轮子不再响应任何命令。
  ⚠ 这会**锁存 FAULT**，解除办法就是第 4 节那条：**先松开 S0，再按 S4**。
  真上车之前必须亲手做过一次这个动作 —— 不要等到事故现场才第一次按它。

### 步骤 4：标定 `wheel_separation_scale`

**这一步不做，后面的导航一定会偏。** 已经验过的后果：未标定时
odom 报 `(0.014, 0.999)` 完全自洽，而车实际在 `(0.413, 0.910)` —— **偏 42 cm**。
原因是每一段由 FPGA 数编码器闭环、非常准，但"下一段往哪走"是主机按里程计
算出来的，而航向尺度错了，折线整体就偏斜。

* 方法、公式、判据、以及"内轮低于约 7 RPM 会滑行，那样测出来的值不可信"
  这条警告，**完整照抄 [`real_robot_params.md`](real_robot_params.md) 第二节**。
* 回传：`θ_odom`、`θ_true`、算出来的 `scale`，以及用的是哪个角速度。

### 步骤 5：雷达

1. 定型号 → 2. 装好、供电、装驱动 → 3. **体检** → 4. 填安装位姿：

```bash
./docker/run.sh ros2 run my_robot_description scan_check.py
# 判据：结尾「雷达这段通了 ✓」
```

> ⚠ **雷达驱动也必须跑在容器里**（宿主是 Ubuntu 22.04，装不了 Jazzy；而且
> `/scan` 要和 Nav2 在同一个 DDS 域里才看得见）。容器目前只挂了**一个**串口设备
> （底盘那个），所以顺序是：**先把你雷达的接口形态告诉软件侧 → 我们改
> `docker/run.sh` 支持同时挂两个设备 → 你再装驱动**。这一步会卡住，不要自己硬来。

它会报：frame_id、TF 能不能查到（`base_link → laser_frame` 的实际 xyz/yaw）、
发布频率、角度范围、点数、NaN/inf、以及**有没有一整片无回波的扇区**
（自遮挡会让 costmap 在这些方向永久留一个贴着的障碍，表现为"车原地转圈、
报 no valid path"）。

体检过了之后，把**卷尺量出来的**安装位姿填进
`urdf/acg720_real.urdf.xacro` 的 `lidar_mount_x/y/z`，同时把雷达参数
（点数、视场角、量程、频率）填进同文件的 xacro property —— 哪一项要连带改哪个
配置文件，[`real_robot_params.md`](real_robot_params.md) 第五节有完整对照表。

要提供的雷达信息清单（**装之前**就该给软件侧，否则等于白装）：

| # | 要什么 | 为什么 |
|---|---|---|
| 1 | 型号 | 决定驱动包与参数 |
| 2 | 接口形态（USB / UART 转接板） | 是不是**第二个** `/dev/ttyUSB*`（容器只挂了一个底盘设备，要一起改） |
| 3 | 安装位姿 x/y/z + yaw（卷尺量） | 填 URDF；错了地图就整体歪 |
| 4 | 光心离地高度、扫描平面高度 | 决定能不能看到桌腿/台阶 |
| 5 | 遮挡角范围（车体挡住的角度） | 不排除就是幽灵障碍 |
| 6 | 供电是否与电机同电池/共地 | 电机 PWM 干扰会让雷达丢点、出现假近距离点 |
| 7 | 谁负责装 ROS 2 驱动包 | aarch64 上经常要源码编译，最容易掉到地上 |

### 步骤 6：建图

```bash
# 边推车边走一圈（真机上 sim:=false）
./docker/run.sh ros2 launch my_robot_description nav.launch.py sim:=false slam:=true
# 保存地图（命令与参数见 slam.md）
```

* **判据**：地图闭合、墙是单线（**双线**说明位姿或时间戳有问题）、没有明显扭曲。
* 细节与两处"不改就一定建错"的配置：[`slam.md`](slam.md)。

### 步骤 7：定位 + 导航联调（**这一步一起做**）

AMCL + Nav2 + 分段导航的联调需要软件侧在场或远程：
`NavigateToPose` → 路径 → 拆成 4A 段 → 逐段下发这条链路的桥正在写。
在此之前，用 `control_mode:=position` + 手工发目标点也能跑通"定点导航"。

---

## 6. 出问题的第一反应（按现象查）

| 现象 | 先看什么 |
|---|---|
| 车不动，日志无错 | ① 是否 FAULT（`real_bringup_check` 第 4 节）② 目标是不是在订阅匹配前发出去的 ③ SEQ 是否需要对齐 |
| 车走走停停，状态机却显示 RUNNING | 运动期间发了 HEARTBEAT（应该重发 MOVE） |
| 车走到半路停住且**不报错** | `done` 边沿检测失效或遥测断了 —— 回传 `diagnostics` |
| 目标刚下发就报失败、车一动不动 | 段被命令限速挡掉，被误算成失败（我们已修，若复现请回传日志） |
| 底盘彻底不理命令 | FAULT 自锁 → 松 S0 再按 S4 |
| 导航整体偏斜 | `wheel_separation_scale` 没标定 |
| costmap 里有个贴着自己的障碍 | 雷达自遮挡（`scan_check` 第 5 节） |

完整排查表：[`position_navigation.md`](position_navigation.md) 第 8 节、
[`acg720_protocol.md`](acg720_protocol.md)。

---

## 7. 做完请回传这些（否则软件侧闭不了环）

| # | 回传什么 | 怎么拿 | 用来干什么 |
|---|---|---|---|
| 1 | 首启日志：`build_id` + `state` + `fault` | `real_bringup_check --arm` 的输出 | 确认位流与链路 |
| 2 | **实际烧的 `.fs` 的 SHA256** | `sha256sum xxx.fs` | 交接包只登记了一个位流 |
| 3 | 标定数据：`θ_odom`、`θ_true`、`scale`、ω | 步骤 4 | 填 `wheel_separation_scale` |
| 4 | 一段 **30 s 的 bag**（`/scan` + `/odom` + `/tf`） | 容器里 `ros2 bag record -o /ws/bag_lidar /scan /odom /tf`（`/ws` 就是板子上的 `~/sim_nav_ws`，所以包会直接落在仓库目录里） | 我们能在自己机器上反复重放调 SLAM/AMCL，不用占车 |
| 5 | `/acg720/diagnostics` 的输出 | `ros2 topic echo /acg720/diagnostics` | `crc_errors`/`frame_errors`/`dropped_bytes` 判断电气质量 |
| 6 | 台架方向/轮序的观察 | 步骤 3 | 确认机械与协议口径一致 |

---

## 8. 工具清单（都已经写好并验证过）

| 脚本 | 什么时候用 | 判据 |
|---|---|---|
| `platform_check.py` | 换机器/第一次跑 | 「没有硬阻塞 ✓」 |
| `real_bringup_check.py` | **接上 FPGA 后第一件事** | 「接线这一段通了 ✓」 |
| `scan_check.py` | 雷达装好之后 | 「雷达这段通了 ✓」 |
| `test_bringup_smoke.py` | 怀疑"软件栈到底还能不能跑" | 32/32，exit=0 |
| `test_position_nav_hil.py` | 动了分段导航相关代码 | 全绿（65 项） |
| `test_hil_end_to_end.py` | 动了协议/驱动 | 全绿（59 项） |
| `acg720_simulator.py --dry-run` | 没有硬件时验虚拟车自己 | 「[dry-run] 自检全部通过」 |

用法与判读口径：[`tools.md`](tools.md)（前五个验收脚本）、
[`acg720_protocol.md`](acg720_protocol.md) 第 8 节（虚拟车）。

---

## 9. 现在还没确认的事（欢迎顺手回答）

| 项 | 谁去测 | 测完填哪 |
|---|---|---|
| 真车有效轮距（打滑修正） | 你，步骤 4 | 驱动的 `wheel_separation_scale` |
| 轮径 / 轮距 / 轴距真值 | 你，卷尺 | `real_robot_params.md` 第〇节（当前都是【待确认】） |
| 烧的是哪个位流 | 你，步骤 2 | 本文档第 7 节回传 |
| IMU 到底装了没 | `real_bringup_check` 第 6 节自动判 | 若装了，`/imu` 由软件侧补 |
| 雷达型号与全部参数 | 你 | `real_robot_params.md` 第四节 |
| 编码器 CPR 是否真是 1320 | 手转一圈对数 | 已由 RTL 注释与"0.5 m = 3232 计数"双向印证，仍建议实测 |
| 内轮低速滑行下限（约 7 RPM） | 台架上观察 | 影响标定与转弯精度 |

---

## 10. 想继续往下看

```
docs/README.md                文档地图（该读哪份）
real_robot_plan.md            实车迁移计划：还差什么、按什么次序、安全怎么保证
acg720_protocol.md            ★ 真协议逐字段 + 驱动节点 + 虚拟车（要对 FPGA 就查这份）
position_navigation.md        ★ 不改 FPGA 的连续移动（0x05 POSITION 折线）用法与实测
fpga_change_request.md        ★ 要让 FPGA 支持连续圆弧运动，要改哪里（给 RTL 的人看）
real_robot_params.md          ★ 标定手册：逐项怎么量、填哪
slam.md / nav2.md             建图与导航的配置细节与坑
tools.md                      验收脚本的用法与判读
```
