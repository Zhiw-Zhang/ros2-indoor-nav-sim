# ACG720 串口协议与驱动层

这份文档讲**上位机怎么和 ACG720 底盘的 FPGA 说话**，以及本工程为此做了哪些代码。

它和另外两份文档的分工：

| 文档 | 讲什么 |
|---|---|
| 本文 | 串口协议、驱动节点、虚拟车、怎么在没有硬件时验证 |
| [`real_robot_plan.md`](real_robot_plan.md) | 整体搬迁顺序：还差什么、按什么次序做、安全怎么保证 |
| [`real_robot_params.md`](real_robot_params.md) | 逐项参数怎么量、填进哪个文件 |

---

## ⚠ 先读这一条：payload 字节布局是**本工程补齐的**，不是交接包给的

交接包里关于串口的**全部**信息只有一句话（`小车数据总览_实测状态分级.md` 第 4.4 节）：

> 串口帧：帧头 `A5 5A`；0x05 POSITION 位置命令；0x06 心跳（100 ms，300 ms 超时）；
> 0x84 遥测（20 Hz）。

也就是说，交接包给了**帧边界、命令字、周期**，没有给：

* 长度字段 / 校验方式 / 字节序
* 每个命令字 payload 里字段的顺序、类型、单位
* 遥测里四轮数据的排列顺序

所以本工程**自行定义**了一套完整布局（见下）。它满足交接包所有已知约束，
并且与交接包 4.1–4.4 节的物理量一一对应。

> ### 拿到 FPGA 源码/协议文档后必须做的事
>
> 1. 只改 **`src/acg720_protocol.py`** 一个文件——驱动节点、虚拟车都从它取布局，
>    不需要动别处。
> 2. 跑 `python3 -m pytest src/my_robot_description/test/ -v`。
>    其中 `test_crc16_modbus_golden_vector` 会告诉你校验算法对不对
>    （如果 FPGA 用的是 CRC-CCITT 0x1021，这个测试会红）。
> 3. 跑虚拟车自检：`python3 src/acg720_protocol.py` 的姊妹脚本
>    `acg720_simulator.py --dry-run`，它不依赖硬件就能确认收发双向通。
> 4. **在此之前，不要拿本协议去接真车电机。** 帧布局不对的后果是
>    底盘按错误速度跑，而日志上一切正常。

---

## 一、帧格式

```
┌──────┬──────┬──────┬───────┬─────────┬──────────┐
│ 0xA5 │ 0x5A │ TYPE │  LEN  │ PAYLOAD │ CRC16 LE │
│  1B  │  1B  │  1B  │  1B   │  LEN B  │    2B    │
└──────┴──────┴──────┴───────┴─────────┴──────────┘
                  └──────── CRC 覆盖范围 ────────┘
```

| 项 | 取值 | 说明 |
|---|---|---|
| 帧头 | `A5 5A` | 交接包给出 |
| TYPE | 命令字，见下表 | |
| LEN | payload 字节数，0–64 | |
| PAYLOAD | 小端序 | |
| CRC16 | CRC-16/MODBUS，**小端** | 多项式 0x8005 反射式，init=0xFFFF，xorout=0 |

**为什么用 CRC 而不是简单校验和**：帧头只有 2 字节、长度只有 1 字节，一旦丢字节，
校验和很容易被误判成合法帧；底盘数据被误判的后果是车按错误速度跑。
按 115200 波特率、20 Hz、每帧约 25 字节算，CRC 占用带宽不到 5%。

**CRC 覆盖范围**：`TYPE + LEN + PAYLOAD`，**不含帧头**。测试向量：
`crc16(b"123456789") == 0x4B37`。

---

## 二、命令字

低半区（`< 0x80`）是**下行**（上位机 → FPGA），高半区是**上行**（FPGA → 上位机）。

| TYPE | 方向 | 名称 | 周期 | 来源 |
|---|---|---|---|---|
| `0x01` | 下行 | SET_VELOCITY 四轮目标 RPM | 由 `/cmd_vel` 触发 | 本工程新增 |
| `0x05` | 下行 | POSITION 位置/定点旋转 | 按需 | **交接包** |
| `0x06` | 下行 | HEARTBEAT 心跳 | **100 ms** | **交接包** |
| `0x83` | 上行 | STATUS 状态/心跳应答 | 20 Hz | 本工程新增 |
| `0x84` | 上行 | TELEMETRY 遥测 | **20 Hz** | **交接包** |
| `0x85` | 上行 | POSITION_DONE 位置回报 | 按需 | 本工程新增 |
| `0x07` | 下行 | SET_CONFIG（**预留，未实现**） | — | 本工程新增 |

### 2.1 `0x01` SET_VELOCITY（8 字节）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | int16 | 目标 RPM **A（右前）** |
| 2 | int16 | 目标 RPM **B（左前）** |
| 4 | int16 | 目标 RPM **C（左后）** |
| 6 | int16 | 目标 RPM **D（右后）** |

轮位约定与交接包一致：**A=右前，B=左前，C=左后，D=右后**。
有符号，负值 = 反转。

### 2.2 `0x05` POSITION（8 字节）

| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | int32 | `counts` | 目标编码器计数，**带符号** |
| 4 | int16 | `rpm` | 巡航 RPM（协议范围 8–40） |
| 6 | uint8 | `mode` | 0=直线，1=原地旋转 |
| 7 | uint8 | `timeout` | 超时，单位 **50 ms 采样**（240 ≈ 12 s） |

⚠ `counts` 的换算依赖 **CPR = 1320** 与**轮径 65 mm**，两者都还是【待确认】。

### 2.3 `0x06` HEARTBEAT（1 字节）

序号 `seq`，uint8，回绕。FPGA 侧 **300 ms** 无心跳进安全态。

### 2.4 `0x84` TELEMETRY（34 字节）

| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | uint32 | `seq` | 递增序号，用来测丢帧 |
| 4 | uint32 | `uptime_ms` | FPGA 上电毫秒数 |
| 8 | int16 ×4 | `counts[4]` | **本采样区间的编码器增量**，A,B,C,D |
| 16 | int16 ×4 | `rpm[4]` | FPGA 自己换算的 RPM，A,B,C,D |
| 24 | uint16 ×4 | `duty[4]` | 四路 PWM duty（0–5000，上限 2700） |
| 32 | uint16 | `flags` | 状态旗标 |

格式串 `"<II4h4h4HH"`，`calcsize = 34`。字段偏移由
`test_telemetry_field_offsets` 逐个钉死。

`flags` 位定义：

| 位 | 名称 | 含义 |
|---|---|---|
| 0 | `HEARTBEAT_TIMEOUT` | FPGA 认为上位机心跳超时，已进安全态 |
| 1 | `ESTOP` | 急停（S0）按下 |
| 2 | `STALL` | 卡死保护（duty≥1800 且实测≤3 RPM 连续 10 采样） |
| 3 | `OVERSPEED` | 实测 > 400 RPM |
| 4 | `TARGET_REJECT` | 目标 > 200 RPM，被拒绝 |
| 5 | `ENCODER_INVALID` | AB 双位同时变化，出现非法跳变 |
| 6 | `LOW_SPEED_WARN` | 低速警告（内轮目标低于约 7 RPM 跟踪不上） |

### 2.5 `0x83` STATUS（8 字节）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | uint8 | `heartbeat_echo` 回显心跳序号 |
| 1 | uint8 | `accepted`（1=接受，0=拒绝） |
| 2 | uint8 | `reject_code` |
| 3 | uint8 | pad，恒为 0 |
| 4 | uint32 | `uptime_ms` |

`reject_code`：0=OK，1=`TARGET_GT_200RPM`，2=`REVERSE_PROTECTION`，3=`ESTOP_ACTIVE`，4=`PARAM_OUT_OF_RANGE`。

> **关于第 3 字节那个 pad**——这一处花了几轮才定下来，值得写清楚，
> 因为它是"上位机和 FPGA 对不上"的典型来源。`struct` 会在字段之间**隐式插入对齐填充**，
> 而且规则不直观：
>
> | 写法 | calcsize | 问题 |
> |---|---|---|
> | `"<IBBI"` | **14** | uint32 被对齐到 4 字节边界，隐式插了 4 字节 |
> | `"<IBB3xI"` | 13 | uint32 落在偏移 6 |
> | `"<IBBxI"` | **11** | 开头的 `I` 让整个结构带 4 字节对齐，`x` 之后又被插 padding |
> | `"<4BI"` | **8** ✓ | 把 pad 写成第 4 个 `B`（恒 0），位置完全由字段决定 |
>
> 教训：`struct` 的 `x` 是"跳过 1 字节"，**不是**可以随便放的填充；只要格式串里
> 出现过 4 字节字段，后续字段就可能被隐式对齐。所以最稳的写法是**不用 `x`**。
> `test_status_layout_is_explicit_and_c_aligned` 把 8 字节和"uptime 在偏移 4"钉死了。

**为什么需要 `accepted`**：交接包 4.3 节写明 FPGA 会拒绝 >200 RPM 的目标。
如果上位机不知道目标被拒，就会出现"发了速度、车没动、日志却显示一切正常"。

---

## 三、运动学与那个必须标定的系数

### 3.1 轮位 → 左右 → 车体

交接包 4.4 节给出的 FPGA 归一化是：

```
前进 = (A+B+C+D)/4          左旋 = (A−B−C+D)/4
```

展开成左右两侧：

```
左轮平均 v_L = (B+C)/2       右轮平均 v_R = (A+D)/2
前进  = (v_L+v_R)/2          左旋  = (v_L−v_R)/2
```

所以这是一台标准 skid-steer，左右两轮差速模型直接可用，没有几何歧义。
单元测试 `test_wheels_to_lr_matches_handoff_normalization` 和
`test_f1_acceptance_case_reproduces_handoff_numbers` 用交接包 F1 验收里
真实出现过的目标值 `(30,40,40,30)` 做了回归。

### 3.2 `wheel_separation_scale`：真机上必须由**上位机**打进去

skid-steer 原地转时四轮横向刮擦，**车体真实转角小于差速模型的预测**。
仿真里实测 `odom/真值 = 1.36`（即"编码器以为转 1 圈，车实际只转了 0.73 圈"）。

关键在于：**ACG720 的 FPGA 不知道这个系数**（它内部只有几何轮距）。
仿真里这个修正塞在 `gz-sim-diff-drive-system` 插件的 `<wheel_separation>` 里，
真机上没有那个插件，所以必须由驱动节点在**两侧同时**打进：

```
下发：Δv = ω_cmd · b_eff    →  ω_true = ω_cmd      （控制增益对）
上报：ω_odom = Δv / b_eff   →  ω_odom = ω_true     （里程计尺度对）
```

其中 `b_eff = 轮距 × scale`。

> **⚠ 真机的 scale 必须重量，不要沿用仿真的 1.36。**
> 仿真那个 1.36 是 ODE 物理引擎 + `wheel_mu_lat=0.5` 的产物，
> 和真车轮胎、地面、负载都无关。真机初始值填 **1.0**，
> 按 `real_robot_params.md` 第二节标定。

---

## 四、驱动节点 `acg720_driver`

它是仿真里那个 gz 插件的**替代品**，接口刻意与仿真保持同名，这样
`nav2_params.yaml` 里几十处引用一个字都不用改：

| 仿真（gz 插件） | 真机（本节点） |
|---|---|
| `<topic>cmd_vel</topic>` | 订阅 `/cmd_vel` |
| 差分运动学积分 | `drive_kinematics.SkidSteerGeometry` |
| 发布 `/odom` + `odom→base_link` | 同样，**但积分源是编码器计数** |
| 内部真值位姿 | 没有（真机不存在） |

### 4.1 发布 / 订阅

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | Nav2 经 `collision_monitor` 后的输出 |
| 发布 | `/odom` | `nav_msgs/Odometry` | 20 Hz（跟随遥测） |
| 发布 | `/tf` | `odom → base_link` | |
| 发布 | `/acg720/telemetry_raw` | `std_msgs/String` | 原始帧内容，人可读，标定用 |
| 发布 | `/acg720/diagnostics` | `std_msgs/String` | 1 Hz 健康度 |
| 发布 | `/acg720/diagnostics_latched` | `std_msgs/String` | 启动结果（latched） |

刻意**不**用自定义 msg：现场 `ros2 topic echo` 就能直接看懂，也不需要重新 build。

### 4.2 位姿为什么用**计数**积分，而不是 FPGA 上报的 RPM

交接包明确写了 CPR=1320 和 RPM 换算公式都还是【待确认】。RPM 是 FPGA 用这个
待确认公式算出来的**二手值**：

* 用 RPM 积分 → 公式一错，位姿误差**随时间累积**，表现为"地图越走越歪"
* 用计数积分 → 计数是一手的；尺度错了只是**整体比例**错，可以靠标定一次修好

所以：**计数 → 位姿**，RPM 只进 `/acg720/telemetry_raw` 供**对比诊断**——
两者一比就能反推 CPR 对不对。

### 4.3 安全相关的三个行为

这三条是仿真里**不存在**、真机必须有：

1. **`/cmd_vel` 超时停车**（`cmd_vel_timeout`，默认 0.5 s）。
   比 FPGA 自己的 300 ms 心跳超时更靠前一道。上位机崩了车必须停。
2. **心跳使能开关**（`send_heartbeat`，默认 true）。
   置 false 可在线验证"FPGA 安全态真的会触发"——上车前必查的一条。
3. **失败要吵**：串口打不开、CRC 持续错、目标被 FPGA 拒绝、低速跟踪不上，
   全部打 WARN/ERROR。否则症状是"launch 起来了、话题也在、车就是不动"。

### 4.4 参数

所有参数都可以用 `--ros-args -p 名:=值` 覆盖。**限幅和超时类参数在用到时才读**，
所以 `ros2 param set` 在线改立刻生效，不需要重启节点。

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | 串口设备 |
| `baudrate` | 115200 | |
| `wheel_radius_m` | 0.0325 | 【待确认】65 mm 轮径 |
| `wheel_separation_m` | 0.200 | 【待确认】200 mm 轮距 |
| `wheelbase_m` | 0.185 | 【待确认】185 mm 轴距 |
| `counts_per_rev` | 1320.0 | 【待确认】修正 2 |
| `wheel_separation_scale` | **1.0** | ★ 必须实车标定，**不要用 1.36** |
| `max_linear_vel` | 0.20 | m/s，比仿真的 0.5 保守 |
| `max_angular_vel` | 1.0 | rad/s |
| `max_wheel_rpm` / `min_wheel_rpm` | ±40 | 协议取值范围 8–40 |
| `rpm_below_tracking_floor` | 7.0 | 交接包 F1 实测的低速下限 |
| `cmd_vel_timeout` | 0.5 | s |
| `heartbeat_period` | 0.1 | s，协议要求 |
| `send_heartbeat` | true | 台架验证安全态时才关 |
| `publish_tf` | true | |
| `reconnect_period` | 1.0 | s，串口掉线重连 |
| `dry_run` | false | 只算不发，无硬件时干跑 |

---

## 五、没有硬件也能验证：虚拟 ACG720

`acg720_simulator.py` 创建一个 **PTY（伪终端）对**，把 slave 路径给驱动节点：

```
虚拟车（master fd） ←── 内核里连着 ──→  /dev/pts/N ←── 驱动节点
```

两边走的是**真实字节流**、真实 pyserial 配置。所以这一层真能验证：
帧编解码、CRC、流式重新同步、串口读写、**命令超时停车**、运动学符号与尺度、
以及下面两个真车已知缺陷。

**不能验证**（它也不假装）：电气、真实电机响应、真实编码器、轮胎摩擦、EMI。

### 5.1 刻意复刻的真车缺陷

依据是交接包的 F1 验收报告：

| 缺陷 | 复刻方式 | 为什么必须复刻 |
|---|---|---|
| 内轮目标 < ~7 RPM 时车以 7 RPM 滑行 | `tracking_floor_rpm=7.0` | 整个避障减速段都在这个区间；不复刻就会在真车上才发现"让它慢慢挪它却冲出去" |
| skid 侧滑 | `ground_truth_separation_scale=1.36` | 默认驱动侧 1.0，所以**默认配置就能看到 1.36 倍偏差**，跑标定能把误差收掉 |
| 拒绝 >200 RPM 目标 | `max_target_rpm=200` + `0x83` 回报 | 验证驱动会把拒绝如实报出来而不是静默 |
| 心跳 300 ms 超时进安全态 | `heartbeat_timeout_s=0.30` | 上车前必查 |

### 5.2 用法

```bash
# 自检：不依赖 ROS，验证 PTY 双向通道 + 协议 + 两个缺陷模型
python3 src/my_robot_description/src/acg720_simulator.py --dry-run

# 拉起虚拟车并打印 /dev/pts/N（然后自己把驱动指过去）
python3 src/my_robot_description/src/acg720_simulator.py --no-driver

# 端到端 HIL：真驱动节点 + 真 PTY，30 项断言
python3 src/my_robot_description/test/test_hil_end_to_end.py
```

---

## 六、测试覆盖了什么

| 文件 | 内容 | 需要 PTY | 规模 |
|---|---|---|---|
| `test/test_protocol_and_kinematics.py` | 纯函数：CRC、帧往返、流式重同步、运动学、位姿积分 | 否 | 50 例 |
| `test/test_hil_end_to_end.py` | 集成：真驱动节点 + 真 PTY | **是** | 30 项 |
| `acg720_simulator.py --dry-run` | 通道自检 + 缺陷模型自检 | **是** | 4 项 |

```bash
# 纯函数部分（任何机器都能跑，毫秒级）
python3 -m pytest src/my_robot_description/test/test_protocol_and_kinematics.py -v

# 集成部分（要能访问 /dev/ptmx）
python3 src/my_robot_description/test/test_hil_end_to_end.py
```

HIL 里有价值的几项：

| 测试 | 断言 |
|---|---|
| 直行 | 0.15 m/s × 2 s → 位移 0.27 m、无横向漂移、四轮目标一致 |
| 旋转方向 | 正 ω → **左轮反转、右轮正转**（最容易搞反的一项，搞反了原地转方向相反） |
| 未标定基线 | `scale=1.0` 时 `θ_odom/θ_true = 1.398`（对应真值 1.36） |
| **标定闭环** | `scale=1.36` 时 `θ_odom/θ_true = 0.9998`，误差 <0.02% |
| 命令超时 | 停发 `/cmd_vel` 0.8 s → 下发零速、车轮停转 |
| 心跳超时 | `send_heartbeat:=false` → FPGA 侧进安全态、轮速清零 |
| 低速下限 | 内轮目标 2.94 RPM → 实际稳定 7.00 RPM，且驱动发出告警 |
| 目标拒绝 | 441 RPM 目标 → FPGA 拒绝回报、`TARGET_REJECT` 置位、轮速未达 400 |
| 串口重连 | 关掉串口后自动重连成功 |

---

## 七、下一步（本层还没做的）

| 项 | 说明 |
|---|---|
| **payload 布局核实** | 见本文开头的警告。这是接真车前的**第一阻塞项** |
| 位置环 0x05 未闭环验证 | 虚拟车只回报"被新命令打断"，没有实现真正的定点闭环；真机的位置环精度要实车验收 |
| `0x07` SET_CONFIG | 预留未实现（改 FPGA 内部限速/阈值） |
| 换向保护 0x83 code=2 | 协议里定义了，驱动尚未针对它做重试策略 |
| 雷达驱动 | 用户已确认会加 2D 雷达（LD19 / RPLIDAR A1 级）。本层只做底盘，雷达驱动是独立一块 |
