# 分段导航：用 0x05 POSITION 在**不改 FPGA** 的前提下连续移动

> **一句话**：位置环不受 3.0 s 连续运动上限约束（`chassis_safety_fsm.v:57-58`
> 的 `!position_active`）。所以把长距离拆成一串 `0x05 POSITION` 段，就能在
> **现有位流**上让车连续走。代价是**没有圆弧** —— 位置环只有四个动作，
> 轨迹只能是"转—直行—转"的折线。

它是 [`fpga_change_request.md`](fpga_change_request.md) 里 P0 变更**落地之前**的
正经替代方案，也是当前唯一能跑连续导航的路径。

---

## 1. 它解决的是什么问题

`0x02 MOVE` 连续跑满 3.0 s 就会锁存 `FAULT_RUN_LIMIT`，之后只能断电或
「松开 S0 再按 S4」复位。而且 `run_age` **不会**被重发命令清零（只在离开
RUNNING 时清，`chassis_safety_fsm.v:74-77`），所以"高频重发 MOVE 续期"是无效的，
"周期 STOP 再 MOVE"是交接包明令禁止的。

位置环走的是另一条判据：

```verilog
// chassis_safety_fsm.v:57-58
assign run_limit_reached = !position_active &&
                           (run_age >= RUN_LIMIT_CYCLES - 32'd1);
```

`position_active` 就是 `position_busy`（`top.v:232`）。位置段运行期间判据恒为假，
所以**单段可以跑很久** —— 实测单段连续 RUNNING 峰值 **8.26 s** 而 `fault=NONE`
（见第 6 节）。

---

## 2. 它做不到什么（都是 RTL 硬约束，不是取舍）

| 做不到 | 依据 | 后果 |
|---|---|---|
| **圆弧 / 连续曲率** | 位置环只认 4 个动作码：1 前进、2 后退、5 原地左旋、6 原地右旋（`chassis_position_controller.v:44-47`，经 `chassis_command_rx.v:104-106` 校验） | 轨迹是折线。走曲线只能靠"缩短直线段 + 频繁小幅修正航向"逼近 |
| **一段跑任意长** | 段内超时 `TIMEOUT_TICKS=240` × 50 ms = **11.95 s**（`:20`） | 段长必须按转速反算上限。见 `position_segments.max_counts_for_rpm` |
| **运行中改段 / 抢占** | 启动条件是 `start_valid && !busy`（`:177`） | 运行中再发 POSITION **什么都不会发生**：既不重启也不中止。必须等 `done` |
| **段与段无缝衔接** | 每段结束要经过 `POS_SETTLE`（等四轮都停稳，`SETTLE_TICKS=3`） | 每段末尾都有一个"停稳"停顿，所以平均速度明显低于单段转速 |
| **位置环期间用手动目标调速** | `top.v:152` `target_rpm = position_busy ? position_target_rpm : manual_target_rpm` | 段进行中 MOVE 被无视；而段**一结束**残留的手动目标会立刻接管 |

### 精度与速度的实测口径

| 项 | 值 | 出处 |
|---|---|---|
| 每段位置残差 | ≤ 32 计数 ≈ **4.9 mm** | `STOP_TOL_COUNTS = 32`（`:19`） |
| 到点精度（实测） | **1~2 mm** | 第 6 节 `[1]`/`[2]`：目标 1.000 → 实测 0.999 |
| 直线段长上限 @30 RPM | 4732 计数 ≈ **0.73 m** | `30/60×1320×11.95×0.6` |
| 转弯段上限 @20 RPM | 3154 计数 ≈ **279°** | 同上，按 `rotation_to_position_counts` 折算 |
| 平均行进速度（实测） | **≈0.087 m/s** | 2.5 m 用了 28.86 s（含每段斜坡与停稳） |

> 平均速度**远低于** `max_linear_vel`（0.20 m/s）。这不是配置问题：每段都要
> 从 8 RPM 斜坡上去（`RAMP_STEP_RPM=4` 每 50 ms），末尾还要在慢速区
> （≤400 计数降到 ≤15 RPM）、爬行区（≤160 计数降到 8 RPM）里磨过去，
> 再加停稳。**做时间预算时必须按 0.08~0.1 m/s 算，不能按 0.2。**

---

## 3. 怎么用

### 起节点

```bash
# 虚拟底盘上先试（不需要硬件）
ros2 run my_robot_description acg720_simulator          # 印出 /dev/pts/N

# 真机/虚拟底盘都用同一个开关
ros2 run my_robot_description acg720_driver.py --ros-args \
  -p port:=/dev/ttyUSB0 -p control_mode:=position
```

或走 launch（参数已接好）：

```bash
ros2 launch my_robot_description robot.launch.py nav:=false control_mode:=position
```

### 派目标

```bash
# odom 系，到点即可（不约束朝向）
ros2 topic pub --once /acg720/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 1.0, y: 0.5}}}'

# 到点并且朝向 -90°
ros2 topic pub --once /acg720/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 0.6, y: 0.6},
    orientation: {z: -0.7071, w: 0.7071}}}'
```

RViz 里 remap 成 `/goal_pose` 就能用 "2D Goal Pose" 工具：

```bash
ros2 run my_robot_description acg720_driver.py --ros-args \
  -p port:=/dev/ttyUSB0 -p control_mode:=position -r acg720/goal:=/goal_pose
```

⚠️ **两个必须知道的坑**：

1. **目标是默认 QoS（VOLATILE + 深度 1）的一条消息。** 在驱动的订阅者
   还没匹配上之前发出去会被**静默丢掉** —— 现象是"话题在、发了没反应、
   连个警告都没有"。所以要在驱动起来之后再派。测试里是重发到
   驱动确认为止（`test_position_nav_hil.py` 的 `send_goal`）。
2. **ROS 里 `PoseStamped` 的默认四元数是 `(0,0,0,1)`，也就是"朝向 +x"，
   不是"未指定"。** 只有显式发全零四元数 `(0,0,0,0)` 才表示"不约束朝向"
   （且必须 `goal_uses_yaw:=true` 才会去读朝向）。这一条真的踩过：
   一个只想"到点即可"的测试没设四元数，于是目标实际含义变成
   "到点并转回 +x"，车老实转了 90° 又转回来 —— 是**测试错了**。

### 看状态

`/acg720/segment_status`（`std_msgs/String`，**latched**，后连上的也能看到）
的第一段是状态词，后面是 `key=value`：

| 状态词 | 含义 |
|---|---|
| `ACCEPTED` | 目标已被接受，开始规划 |
| `ARRIVED` | 到达（位置 + 朝向都满足） |
| `FAILED` | 放弃，`detail=` 里有原因 |
| `ABORTED` | 被中止（人工接管 / 目标被替换） |
| `REJECTED` | 压根没接：底盘已锁死，或位流版本不符 |

实例：

```
ARRIVED goal_seq=1 state=arrived goal=(2.50,0.00) cur=- planned=5 sent=4 \
ok=4 retried=0 aborted=0 sendfail=0 travel=2.498m detail=已到达目标
```

`/acg720/diagnostics` 里也多了 `segments=(...)`、`position_busy=`、
`seg_limits=(max_straight=…,max_turn=…)`。

### 人工接管

`position` 模式下 `/cmd_vel` **不是控制输入**，而是"人工接管"信号：
任何非零值都会立刻中止当前目标并停车（`cmd_vel_overrides_goal`，默认开）。
摇杆永远能压过自主导航 —— 台架调试时这是保命开关。

中终止是靠发 `STOP` 实现的，因为 `top.v:121-125` 里 `position_abort` 认
STOP(0)/MOVE(2)/DISARM(3)/OPEN_LOOP(4) 这四种 kind。

---

## 4. 参数

都在驱动节点上，launch 里同名透传。

| 参数 | 默认 | 说明 |
|---|---|---|
| `control_mode` | `move` | 改成 `position` 才启用本页功能 |
| `goal_topic` | `acg720/goal` | 目标话题（`PoseStamped`） |
| `goal_uses_yaw` | `false` | 是否把目标朝向当约束 |
| `cmd_vel_overrides_goal` | `true` | 非零 `/cmd_vel` 中止目标 |
| `segment_heading_tolerance_rad` | `0.10` | 航向误差大于它就原地转（≈5.7°） |
| `segment_position_tolerance_m` | `0.03` | 离目标小于它就算到位 |
| `segment_straight_rpm` | `30` | 直线段转速。固件只接受 **8–40** |
| `segment_turn_rpm` | `20` | 原地转转速。固件只接受 **8–40** |
| `segment_max_bad_streak` | `3` | 连续几段可重试失败（STALL/TIMEOUT）就放弃 |
| `segment_max_without_progress` | `6` | 连续几段没靠近目标就放弃（防活锁） |

**参数校验是刻意的**：`segment_straight_rpm` / `segment_turn_rpm` 越界、
`segment_heading_tolerance_rad` 小到"指令不动"、`segment_position_tolerance_m`
小到算不出合法段 —— 都会在**建节点时**直接抛异常。宁可起不来，也不要
"起来了但每一段都发不出去"。

---

## 5. 原理：驱动在 position 模式下守的三条规矩

违反任何一条都会出真问题，所以值得写清楚。

1. **段进行中只发 HEARTBEAT。** `top.v:121-125` 把 STOP/MOVE/DISARM/OPEN_LOOP
   当成 `position_abort`，发出去就把当前段**中断**掉。
   HIL 测试直接在 PTY 字节流上断言"整个目标期间 MOVE 出现 0 次、STOP 出现 0 次"。
2. **绝不发 MOVE。** `top.v:152` 的三目运算符意味着段结束那一拍 `busy` 落回 0 后
   `target_rpm` 立刻切回手动目标 —— 若同时在下 MOVE，段一结束车就会带着残留
   目标冲出去。所以 position 模式在 `_send_due_command` 里就**分叉**，
   和 move 模式互斥。
3. **段下发前独立再校验一遍参数。** 越界的 POSITION 会被判 `protocol_fault`，
   而 `top.v:231` 把它 OR 进了安全状态机的锁存 —— 那是**永久锁死底盘**，
   不是"报个错重来"。规划器侧有穷举单测保证不产出非法段
   （`test_position_segments.py::test_segments_always_pass_protocol_encoding`），
   驱动侧是第二道闸。

另外两条实现细节，都是踩出来的：

* **`0x84` 的 `done` 是电平不是边沿。** 固件完成时置 `done=1`，要到**下一段
  start** 才清零（`chassis_position_controller.v:178`/`:201`），而 `0x84` 是
  20 Hz 周期广播的一部分。照单全收就会把上一段的完成帧当成**新段**的结果 ——
  "还没走完就宣布走完"，车停在半路而且不报错。驱动只认上升沿。
* **被命令限速挡掉 ≠ 下发失败。** 限速是 50 ms（防 ACK 单槽撞车），控制拍是
  10 ms，所以"规划好的段被挡掉"每 5 拍就会发生一次。算进失败计数就三次放弃，
  症状是"目标刚下发就报失败、车一动不动"，而且看起来是随机的。现在
  `on_send_deferred` 和 `on_send_failed` 是两条路。

---

## 6. 实测（`test_position_nav_hil.py`，65 项，连跑 3 次稳定）

| 场景 | 结果 |
|---|---|
| 直线 1.0 m | 2 段，真值 **(0.999, 0.000)**，容差 0.03 |
| 直线 **2.5 m** | 4 段，运动 **28.86 s**，真值 **(2.498, 0.000)** |
| ★ 单段连续 RUNNING 峰值 | **8.26 s** 而 `fault=NONE`（MOVE 做不到这件事） |
| 斜向 (0.6, 0.6) + 朝向 −90° | 真值 (0.606, 0.592)，yaw **−89.3°**（误差 0.7°） |
| 目标在正后方 (−0.7, 0) | 段动作码序列 **[5, 1]**（先原地转、再直行），真值 (−0.699, 0.005) |
| 到位后静置 2 s | 又走 **0.1 mm**（没有残留手动目标接管） |
| 人工接管 | 目标立刻 ABORTED，车停住，底盘 `fault=NONE` |
| 位流版本不符 | **明确拒绝**目标，一帧 POSITION 都没发 |

### 未标定时会怎样（这一条很重要）

同一套代码，让"车体真实有效轮距"是驱动认为的 **1.36 倍**（也就是
`wheel_separation_scale` 没标定）：

```
驱动侧 odom 位姿 = (0.014, 0.999)   ← 它认为到了
车体真值         = (0.413, 0.910)   ← 实际偏了 42.2 cm
```

**odom 完全自洽、却离目标 42 cm。** 原因是位置段**不需要里程计**就能精确走完
（FPGA 自己数编码器），所以每段的执行是准的；但"下一段该往哪走"是主机按
里程计算出来的，而航向尺度错了 → 每条直线都走偏 → 折线整体偏斜。

> 所以：**用分段导航之前必须先标 `wheel_separation_scale`**
> （[`real_robot_params.md`](real_robot_params.md) 第二节）。
> 好消息是它对 `counts_per_rev` / 轮径的标定误差**不敏感** —— 那些误差是
> 按比例缩放的，而每段结束都会重新规划，位置残差累积不起来。

---

## 7. 和 FPGA 变更（P0）的关系

| | `control_mode:=move` | `control_mode:=position` |
|---|---|---|
| 运动表达能力 | 直行 + 圆弧 + 原地转（连续） | 只有"转—直行—转"折线 |
| 3.0 s 上限 | **受限** | 不受限 |
| 平均速度 | 快（受 `max_linear_vel` 限制） | 慢（≈0.09 m/s，每段都有斜坡与停稳） |
| 需要 FPGA 变更 | **需要**（P0） | 不需要 |

**两者不是替代关系，是互补的。** P0 落地后，`move` 模式适合快速平滑的路径跟踪；
`position` 模式仍然有用：它每段都由 FPGA 闭环、按编码器计数收敛，
**走固定距离的重复精度比开环速度控制好**（适合"精确停到某个位置"这类动作）。

---

## 8. 排查表

| 症状 | 最可能的原因 |
|---|---|
| 发了目标，`/acg720/segment_status` 停在 `ACCEPTED`，车不动 | 底盘锁死了，或者位流版本不符 —— 看 `detail=` |
| 发了目标，状态话题**什么都没有** | 目标在订阅者匹配前发出去了（见第 3 节坑 1）。重发一次 |
| `REJECTED` + `detail=位流版本不符` | 烧的位流不对（多半是曲线模型那版）。**先别往下走** |
| `FAILED` + `连续 N 段可重试失败（最后一段 STALL）` | 轮子被卡住 / 转速太低带不动 / 负载过重 |
| `FAILED` + `连续 N 段可重试失败（最后一段 TIMEOUT）` | 段太长而转速太低。调小段长上限或调大 `segment_straight_rpm` |
| `FAILED` + `连续 N 段没有靠近目标` | 活锁：通常是 `wheel_separation_scale` 错得离谱，车在原地绕 |
| `FAILED` + `段下发后 …s 没收到 0x84 完成帧` | 完成帧丢了，或固件没启动这一段。看串口 CRC/丢字节计数 |
| 车走到半路停住，**不报错** | `done` 边沿检测失效（本工程踩过）；或遥测断了 |
| 到点后车又自己走了 | 段结束后有残留手动目标 —— position 模式里混进了 MOVE |

---

## 9. 还没做的（诚实的缺口）

* **没有 action server。** 现在只能靠订阅 latched 状态话题来判断"到没到"。
  Nav2 的 behavior tree / `nav2_simple_commander` 期望的是
  `NavigateToPose` action —— 要接 Nav2 得再包一层 action server。
* **不支持路径（waypoint 列表）**，只有单个目标点。
  `SegmentPlanner.plan(pose, target)` 的接口本来就是"给一个目标点、还一段"，
  所以跟随路径只要喂"路径上的前瞻点"即可 —— 但导航器侧（从 Nav2 的
  `FollowPath` 收路径、算前瞻点、处理障碍中止）还没写。
* **没有动态避障。** 一段一旦发出就由 FPGA 走完，主机侧只能在段与段之间
  （每 0.7 m 左右，或 ~7 s）才有机会中止。**安全上必须靠雷达/碰撞检测在
  段间介入**，不能指望段内。
* **精度只在仿真里验过。** 真值轮距、CPR、轮胎打滑都还是【待确认】。
  上车后第一件事是标 `wheel_separation_scale`，然后重跑本页第 6 节。
