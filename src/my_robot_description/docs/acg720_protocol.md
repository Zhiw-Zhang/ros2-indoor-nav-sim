# ACG720 串口协议与驱动层

> **本文档的每一条结论都来自 FPGA 源码，并标注 `文件:行号`。**
> 出处是交接包 `robot_car_handoff_20261006`（**包外，不在本仓库**），
> 具体是 `motor/wifi_car_position_loop_baseline_20261006/`。
>
> ⚠️ **本文件是重写的，不是修订的。** 上一版描述的是本工程**自己发明**的协议
> （帧外层没有 VER/SEQ/LEN、CRC 用 MODBUS、`0x05` 当成"四轮速度"、
> `0x84` 当成"底盘遥测"）。真固件在**第 3 个字节**就会判版本错并把底盘**永久锁死**，
> 所以那一版不是"参数不准"，而是"根本说不上一句话"。旧的 `acg720_protocol.py`
> 已随本次重写删除，取代它的是 [`acg720_protocol_v1.py`](../src/acg720_protocol_v1.py)。

---

## 0. 先读这一节：四条会咬人的硬约束

这四条各自都能在真车上造成"车不动、日志正常"或者"车锁死、只能断电"。
把它们放在最前面，是因为下面所有细节都是为了让这四条不被违反。

| # | 约束 | 依据 | 症状 |
|---|---|---|---|
| 1 | **任何一帧格式错都会把底盘锁死在 FAULT**，只能断电或"松开 S0 再按 S4"复位 | `chassis_command_rx.v:191,208,245,247` + `chassis_safety_fsm.v:88` | 车彻底不动，且**之后所有命令都无效** |
| 2 | **固件接受的 LEN 上限是 8**，不是 `INTERFACE.md` 里写的 48 | `chassis_command_rx.v:208` | 按 `INTERFACE.md` 的草案发 `0x07`（LEN=10）→ 立刻锁死 |
| 3 | **运动期间不能发 HEARTBEAT**：任何非 MOVE 的有效命令都会把运动目标清零 | `top.v:238-281` | 车走走停停，状态机却停在 RUNNING |
| 4 | **ACK 只有单槽**：两条被接受的命令挤进同一个槽会 `tx_overflow` → 也是永久锁死 | `chassis_telemetry_imu_tx.v:236-238` + `top.v:231` | 命令发太快 → 锁死 |

外加一条**能力**约束（不是格式问题）：

| # | 约束 | 依据 | 后果 |
|---|---|---|---|
| 5 | **连续运动（`0x02 MOVE`）最多 3.0 s**，超时锁存 `FAULT_RUN_LIMIT` | `chassis_safety_fsm.v:11,57-58,72-77` | 连续导航今天做不到，只能"分段走" |

---

## 1. 帧格式

**接收与发送同构**：

```
A5 5A | VER=0x01 | KIND | SEQ | LEN | PAYLOAD(LEN) | CRC16(低字节, 高字节)
```

| 字段 | 宽度 | 说明 |
|---|---|---|
| 帧头 | 2 | 固定 `A5 5A` |
| VER | 1 | 固定 `0x01`（`chassis_command_rx.v:45`）。**不符即判非法帧** |
| KIND | 1 | 见第 2 节 |
| SEQ | 1 | 序号，见第 4 节 |
| LEN | 1 | **只算 payload**；>8 判非法（`:208`） |
| PAYLOAD | LEN | |
| CRC16 | 2 | 低字节先发 |

### 1.1 CRC

**CRC-16/CCITT-FALSE**：多项式 `0x1021`、初值 `0xFFFF`、**不反射**、**不异或输出**；
覆盖范围是 **VER..PAYLOAD**（**不含帧头 `A5 5A`**，也不含 CRC 自身）。

* 固件实现：`chassis_command_rx.v:54-70`（逐位）、`:113`（比较）、`:180,187,196,201,206,218`（逐字节累积）
* 自检向量：`crc16(b"123456789") == 0x29B1`
* 对照：**CRC-16/MODBUS** 对同一串数据是 `0x4B37` —— 旧实现用的就是它，算法根本不同

⚠️ 这是最容易搞错的一处。CRC 算错 = 判非法 = **锁死底盘**，不是"这一帧被丢掉"。

### 1.2 接收状态机与自恢复

`chassis_command_rx.v:37-44` 共 9 个状态。有两个容易做错的地方：

* **`ST_HEADER_1` 收到 `A5` 时留在原状态**（`:181-183`）。
  所以 `A5 A5 A5 5A ...` 能自恢复。写成"不是 5A 就回 HEADER_0"会让这种流永久失步。
* **帧内字节间隔超过 100 ms**（`FRAME_TIMEOUT_CYCLES = 5_000_000`，`:11`）→
  丢掉半个帧、`frame_error_count++`、回到 HEADER_0，
  **不置 fault**（`:156-160`）。这是四类非法里**唯一不锁死底盘**的一类。

---

## 2. 命令（主机 → 底盘）

| KIND | 名称 | LEN | payload | 语义校验依据 |
|---|---|---|---|---|
| `0x00` | STOP | 0 | — | `:110-112` |
| `0x01` | ARM | 0 | — | 同上 |
| `0x02` | MOVE | 4 | `mode u8 \| base_rpm u16LE \| ratio u8` | `:97-99` |
| `0x03` | DISARM | 0 | — | `:110-112` |
| `0x04` | OPEN_LOOP | 4 | `mode u8 \| duty u16LE \| 0` | `:100-103` |
| `0x05` | POSITION | 8 | `mode u8 \| max_rpm u8 \| counts u32LE \| 0 u16` | `:104-109` |
| `0x06` | HEARTBEAT | 0 | — | `:110-112` |

取值范围（越界一律**判非法并锁死**，所以在主机侧 `encode_command()` 就拦住）：

| 命令 | 约束 |
|---|---|
| MOVE | `mode ≤ 6`、`base_rpm ≤ 200` |
| OPEN_LOOP | `mode ∈ {1,2}`、`duty ≤ 1400`、第 4 字节必须为 0 |
| POSITION | `mode ∈ {1,2,5,6}`、**`8 ≤ max_rpm ≤ 40`**、`counts > 32`、保留 2 字节必须为 0 |

⚠️ POSITION 的转速检查用的是 **payload[1] 单字节**，不是拼出来的 16 位值 ——
因为第 2 个字节在 POSITION 里属于计数（`:107` 用的是 `payload_rpm_low`）。
把它误当成 16 位 rpm 会把合法帧判成非法。

### 2.1 动作编码（`chassis_motion.v:20-26`）

| mode | 含义 | MOVE | POSITION | OPEN_LOOP |
|---|---|---|---|---|
| 0 | 停 | ✓ | ✗ | ✗ |
| 1 | 前进 | ✓ | ✓ | ✓ |
| 2 | 后退 | ✓ | ✓ | ✓ |
| 3 | 左转（弧线） | ✓ | ✗ | ✗ |
| 4 | 右转（弧线） | ✓ | ✗ | ✗ |
| 5 | 原地左旋 | ✓ | ✓ | ✗ |
| 6 | 原地右旋 | ✓ | ✓ | ✗ |

---

## 3. 遥测（底盘 → 主机，20 Hz）

`TELEMETRY_PERIOD_CYCLES = 2_500_000`（`chassis_telemetry_imu_tx.v:4`）→ 50 ms。
四种周期包**同时**置为待发（`:230-234`），按 `ACK > 0x81 > 0x82 > 0x83 > 0x84`
的优先级逐个发（`:241-297`）。

| KIND | 名称 | LEN | SEQ 的来源 |
|---|---|---|---|
| `0x80` | ACK | 2 | **被 ACK 的那条命令的 SEQ**（`:244`） |
| `0x81` | 底盘遥测 | 38 | 最后被接受的命令 SEQ（`:250`） |
| `0x82` | IMU 转发 | 36 | `imu_sample_counter[7:0]`（`:262`） |
| `0x83` | 编码器诊断 | 42 | `diagnostic_counter[7:0]`（`:279`） |
| `0x84` | 位置遥测 | 16 | **同一个** `diagnostic_counter[7:0]`（`:289`） |

⚠️ **0x81 的 SEQ 不是遥测计数器**，它是"固件最后接受的那条命令的序号"
（payload 偏移 2 里还有一份冗余）。所以**不能拿它判断遥测丢帧**。
同理，0x83 与 0x84 共用同一个计数器，两者的 SEQ 集合是一样的，也不能用来数丢帧。

### 3.1 `0x81` 底盘遥测（38 字节，`:109-131`）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u8 | 安全状态（0=DISARMED 1=ARMED_STOP 2=RUNNING 3=FAULT） |
| 1 | u8 | 故障码（0=NONE 1=ESTOP 2=MOTOR 3=PROTOCOL 4=TIMEOUT 5=RUN_LIMIT） |
| 2 | u8 | `last_command_seq` |
| 3 | u8 | `motor_stby` |
| 4 | 4×s16 | `target_rpm` A,B,C,D |
| 12 | 4×s16 | `speed_rpm` A,B,C,D（实测） |
| 20 | 4×u16 | `duty_cycles` A,B,C,D |
| 28 | 4×u8 | `channel_fault` A,B,C,D |
| 32 | u16 | `crc_errors`（**固件**收到的坏帧数） |
| 34 | u16 | `duplicates` |
| 36 | u16 | `frame_errors` |

### 3.2 `0x83` 编码器诊断（42 字节，`:159-197`）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u16 | `diagnostic_counter` |
| 2 | u32 | `fpga_uptime_ms` |
| **6** | **u32** | **`build_id`** ← **位流版本检测的抓手** |
| 10 | 4×i32 | `position_count` A,B,C,D（**绝对累计**，s32，会回绕） |
| 26 | 4×s16 | `sample_delta` A,B,C,D（最近 50 ms 增量） |
| 34 | 4×u16 | `invalid_transition_count` A,B,C,D |

**`build_id` = `0x20260A02`**（`chassis_telemetry_imu_tx.v:40`）。
厂商的 `position_control.py:110-113` 就是靠它拒绝跑错位流的：
位流版本不对就拒绝执行。本工程的驱动同样在启动时校验，
不符则**拒绝发送任何运动命令**。

`position_count` 是 s32 累计值、**会回绕**，里程计必须按模 2³² 取差量
（`acg720_protocol_v1.delta32`）。

### 3.3 `0x84` 位置遥测（16 字节，`:178-197`）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u8 | flags：bit0 `busy`、bit1 `done`、bits5:2 `result_code` |
| 1 | u8 | meta：bits2:0 位置环状态、bits5:3 `motion_mode` |
| 2 | u16 | `base_rpm`（当前期望转速） |
| 4 | u32 | `target_counts` |
| 8 | i32 | `progress_counts` |
| 12 | u32 | `remaining_counts` |

`result_code`：0=NONE 1=OK 2=ABORTED 3=STALL 4=TIMEOUT 5=REJECTED。

### 3.4 `0x82` IMU 转发（36 字节，`:132-156`）

flags 的位序（RTL 里是 `{...}` 拼接，MSB 先）：bit7 `frame_timeout_seen`、
bit6 `checksum_error_seen`、bit5 `uart_error_seen`、bit4 `stale`、bit3 `fresh`、
bits2:0 `valid_{angle,gyro,accel}`。

⚠️ 台架上没插 IMU，帧照发但 flags 报 **stale**。
虚拟底盘**绝不伪造"有效"的 IMU 数据** —— 宁可让人看到"没有 IMU"，
也不能给一份假姿态。

### 3.5 `0x80` ACK（2 字节）

`state u8 | fault u8`（`:105-106`）。

⚠️ 固件自己的契约：**ACK 只证明帧被解析并接受，不证明动作已执行**。
要确认执行结果得看 0x81 的 `target_rpm` 或 0x84 的进度。

---

## 4. SEQ 规则

```verilog
seq_delta    = seq_hold - last_seq;                                     // :93
sequence_new = !last_seq_valid || ((seq_delta != 0) && !seq_delta[7]);  // :94-95
```

翻译成人话：

| 情况 | delta | 结果 |
|---|---|---|
| 首次通信 | —— | **接受**（`!last_seq_valid`：上电后第一帧来者不拒） |
| 正常前进 | 1..127 | 接受，回 ACK |
| 重复 | 0 | **静默丢弃**：只 `duplicate_count++`，**不回 ACK、不报错** |
| 落后 ≥128 | 128..255 | 同上，静默丢弃 |

**唯一例外是 STOP**：`else if (sequence_new || stop_command)`（`:248`），
序号不新也执行。这是刻意的逃生口。

⚠️ **主机重启后 SEQ 归零不会被自动接受。** 必须先用 0x81 payload 偏移 2 的
`last_command_seq` 重新对齐（`SequenceTracker.resync_from_telemetry`），
否则新序号会被判成"落后"而**静默丢弃** ——
现象是"话题有、驱动在、日志无错、车就是不动"，现场极难查。

同理，**串口重连之后也必须重新对齐**，而且对齐前要先丢掉缓冲区里积压的**旧遥测**
（否则会按一个过期的序号对齐）。

---

## 5. 安全状态机

`chassis_safety_fsm.v`。状态与故障码：

```
状态：DISARMED(0) → ARMED_STOP(1) → RUNNING(2) → FAULT(3)
故障：NONE(0) ESTOP(1) MOTOR(2) PROTOCOL(3) TIMEOUT(4) RUN_LIMIT(5)
```

### 5.1 迁移规则（`:74-175`，顺序即优先级）

1. `!reset_n` → DISARMED（**按住 S4**）
2. `!estop_n || estop_seen` → **FAULT_ESTOP**
3. `state == FAULT` → **保持 FAULT**（自锁，`:88`）
4. RUNNING 且位置环报 `internal_stop` → ARMED_STOP
5. RUNNING 且到 3.0 s 上限 → **FAULT_RUN_LIMIT**
6. `motor_fault` → FAULT_MOTOR
7. `protocol_fault`（含 `tx_overflow`）→ **FAULT_PROTOCOL**
8. armed 且 300 ms 没收到**有效命令** → **FAULT_TIMEOUT**
9. 收到有效命令 → 按状态处理（DISARMED 只认 ARM；ARMED_STOP 与 RUNNING 认
   STOP/ARM/MOVE/OPEN_LOOP/POSITION/HEARTBEAT/DISARM）

### 5.2 三条反直觉但必须照做的行为

**① FAULT 是自锁的。** `else if (state == STATE_FAULT) state <= STATE_FAULT;`
没有任何命令能清除它。唯一出路是 `reset_n`（按住 S4），
而 `estop_seen` 的清零条件还要求**先松开 S0**（`:69-72`）。
→ 恢复动作固定是：**断电，或"松开 S0，再按 S4"**。

**② DISARMED 下非 ARM 命令被静默忽略**（`:120-126`）。
不报错、不置 fault、不回拒绝。现象是"发了命令没反应"。

**③ 300 ms 命令超时对 RUNNING 一样有效。**
所以哪怕在跑位置动作，也必须持续发命令（厂商的 `position_control.py`
每 100 ms 发一条心跳）。**主机的正确喂狗方式是**：

| 状态 | 发什么 |
|---|---|
| 运动（RUNNING） | **重发 MOVE**（它同样刷新 `command_age`） |
| 保持 ARM 但静止 | HEARTBEAT |
| 想让车停 | STOP（RUNNING→ARMED_STOP），然后转心跳 |

⚠️ **运动期间发 HEARTBEAT 会把车刹停**：`top.v:238-281` 的 `active_*`
只认"MOVE 且 mode≠0 且 rpm≠0"（或"OPEN_LOOP 且 duty≠0"），
**其它任何被接受的命令**（含 HEARTBEAT）都走 `else` 分支把目标清零。
状态机却还停在 RUNNING —— 于是表现为"车走走停停、日志一切正常"。

### 5.3 3.0 秒连续运动上限

```verilog
assign run_limit_reached = !position_active && (run_age >= RUN_LIMIT_CYCLES - 1);  // :57-58
// run_age 的更新：
if (!reset_n || (state != STATE_RUNNING)) run_age <= 0;
else if (!run_limit_reached)              run_age <= run_age + 1;                 // :72-77
```

两个后果：

1. **重发命令不清零 `run_age`。** 所以"高频重发 MOVE 就能一直跑"是错的：
   10 Hz 重发也一样在 3.0 s 触顶。（虚拟底盘上有对应的测试钉住这一条。）
2. **位置环期间 `run_age` 照常累积**，被屏蔽的只是"触发"。
   位置动作结束时 `run_limit_reached` 其实已经成立，只是
   `internal_stop` 在优先级链里排在它前面（`:92-96`），
   于是走 RUNNING→ARMED_STOP 而不是 FAULT。
   **这个顺序要是反了，每一次超过 3 秒的位置动作都会把底盘锁死。**

**严禁**用"周期 STOP + 重新 MOVE"绕过这条上限：交接包
`navigation/docs/INTERFACE.md` 明令禁止，而且那会让车每 3 秒急停一次，
既危险又掩盖了真正该做的 FPGA 变更。见 [`fpga_change_request.md`](fpga_change_request.md)。

---

## 6. `0x02 MOVE` 的表达能力：能做什么、不能做什么

### 6.1 展开规则（`chassis_motion.v:32-87`）

```verilog
target_valid = (base_rpm <= 200) && (motion_mode <= 6);
outer_rpm    = base_rpm;
inner_rpm    = base_rpm * turn_inner_ratio_q8 >> 8;   // ← 无符号，只能缩小
```

| mode | 四轮 (A右前, B左前, C左后, D右后) |
|---|---|
| 1 前进 | `base, base, base, base` |
| 2 后退 | `-base, -base, -base, -base` |
| 3 左转 | `base, inner, inner, base` |
| 4 右转 | `inner, base, base, inner` |
| 5 原地左旋 | `base, -base, -base, base` |
| 6 原地右旋 | `-base, base, base, -base` |

### 6.2 它能表达绝大多数 (v, ω)

`(base_rpm, ratio)` 是两个自由参数，正好解两个未知数。设有效轮距 `L`、
`ρ = ratio/256`：

```
内轮 = base·ρ        外轮 = base
v    = base·mps·(1+ρ)/2
ω    = base·mps·(1−ρ)/L
⟹ ρ = (v − |ω|·L/2) / (v + |ω|·L/2)
```

所以**只要 `R = v/ω ≥ L/2`，任意 (v, ω) 都能精确解出来**。
`ρ = 0`（内轮停转、绕内轮转）就是**不反转内轮能达到的最紧弯**，
半径恰是半个有效轮距。

### 6.3 表达不出来的只有两类

| 情况 | 为什么 | 本工程的处理 |
|---|---|---|
| 转弯半径 `< L/2` | 内轮需要**反转**，而 `inner_rpm` 是无符号移位 | 投影成原地转（保留 ω、丢掉 v），并**告警** |
| 倒车转弯（v<0 且 ω≠0） | 固件没有这个动作，mode 2 只有直退 | 投影成直退（保留 v、丢掉 ω），并**告警** |

投影时保留"更接近请求的那一半"，而不是"按峰值轮速换算"：
后者会让车比命令的**更快**，方向是危险的。

`drive_kinematics.twist_to_move()` 负责反解，`move_to_twist()` 负责把
"实际会跑到什么"反算回来（走固件自己的展开逻辑，保证诊断不说谎）。

### 6.4 与 `INTERFACE.md` 草案的关系

`navigation/docs/INTERFACE.md` 提议新增 `0x07 VELOCITY`（**LEN=10**）和
`0x85 NAV_TELEMETRY`（LEN=36），并声明"新增字节协议编号尚未获电机负责人确认，
不得烧录/发送草案到现有固件"。

⚠️ 该文档**没有对着 RTL 核对过**：现有接收机的 LEN 上限是 **8**
（`chassis_command_rx.v:208`），所以 `0x07` 那 10 字节的 payload 会被直接判非法
→ **锁死底盘**。而且 `0x07` **未分配、未实现**，即使 LEN 合法也过不了语义校验
（`command_kind` 只有 3 位，0x08 及以上永远过不了）。

结论：**现状下不需要 `0x07`。** 第 6.2 节说明 MOVE 已经能覆盖连续导航需要的
绝大多数 (v, ω)；真正的拦路虎是 3.0 s 上限（第 5.3 节），
那是**一处策略变更**，不是新增一条命令。

---

## 7. `0x05 POSITION`：今天唯一能真正驱动真车的运动原语

### 7.1 闭环参数（`chassis_position_controller.v:9-17`）

| 参数 | 值 | 含义 |
|---|---|---|
| `MIN_RPM` | 8 | 最低转速 |
| `SLOW_RPM` | 15 | 慢区转速 |
| `RAMP_STEP_RPM` | 4 | 每 50 ms 最多变速 |
| `SLOW_ZONE_COUNTS` | 400 | 剩余 <400 计数进入慢区 |
| `CREEP_ZONE_COUNTS` | 160 | 剩余 <160 计数进入蠕行区 |
| `STOP_TOL_COUNTS` | 32 | 剩余 ≤32 计数认为到位 |
| `SETTLE_TICKS` | 3 | 四轮实测转速为 0 连续 3 拍才算完成 |
| `STALL_TICKS` | 10 | 进度停滞 10 拍判堵转 |
| `TIMEOUT_TICKS` | 240 | **12 s** 总超时（每拍 50 ms） |

`run_age` 对位置动作的屏蔽见第 5.3 节 —— 所以位置动作**可以超过 3 秒**，
这一点与 MOVE 截然不同。

### 7.2 `target_counts` 的口径：是"每个轮子"的计数

```verilog
progress_now = normalized_sum[33:2];     // :78  ← 右移 2 位 = 除以 4
```

而 `normalized_*` 是四轮增量按动作归一化后的和（`:117-141`：
前进四轮取正；原地转右前/右后取正、左前/左后取负）。所以：

```
progress = 四个轮子增量的**平均**     （不是求和！）
```

* **前进**：`target_counts` = 每个轮子滚过的编码器计数
* **原地转**：左右各滚 `target_counts` 个计数

⚠️ 搞错这一条，标定出来的"每米多少计数"会差 **4 倍**。

### 7.3 换算（`drive_kinematics.SkidSteerGeometry`）

```
前进：  counts = d · CPR / (2πr)
原地转：counts = Δθ · L_eff / 2 · CPR / (2πr)
```

其中 `L_eff = wheel_separation_m × wheel_separation_scale`（**有效**轮距，
因为 skid-steer 原地转时四轮横向刮擦，车体真实转角小于几何预测）。

验算：`CPR=1320`、`r=32.5 mm`、`L_eff=200 mm` 时
**0.5 m ↔ 3232 计数**，正好命中交接包示例里的 `POSITION(1, 25, 3232)`。

### 7.4 完成的判据

`busy` 落下 **且** `result_code == OK`（1）。只看 `busy` 会把
`STALL`/`TIMEOUT`/`ABORTED` 当成成功。

---

## 8. 驱动节点 `acg720_driver`

### 8.1 发布 / 订阅

| 方向 | 话题 | 说明 |
|---|---|---|
| 订阅 | `/cmd_vel` | 与仿真里 gz 插件同名，所以 `nav2_params.yaml` 一个字都不用改 |
| 发布 | `/odom` + `odom→base_link` TF | 由 **0x83 的累计计数**积分而来 |
| 发布 | `/acg720/diagnostics` | 1 Hz 人可读诊断（`std_msgs/String`，现场 `ros2 topic echo` 就能看懂） |
| 发布 | `/acg720/diagnostics_latched` | TRANSIENT_LOCAL：回答"这个节点上次启动到底成没成功" |
| 发布 | `/acg720/telemetry_raw` | 0x81 的原始字段，排查用 |

### 8.2 位姿为什么用**计数**积分，而不是上报的 RPM

计数是一手数据：尺度错了也只是整体比例错，可以靠一次标定修好；
而上报的 RPM 是 FPGA 用定点公式
`RPM = (min(|计数|,32767) × 238313) >> 18`（`encoder_quadrature.v:53,105-129`）
算出来的二手值（"RPM = 计数×10/11" 只是它的近似说法），
有量化误差，拿它积分会让误差**持续累积**。

### 8.3 关键参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` / `baudrate` | `/dev/ttyUSB0` / 115200 | |
| `control_mode` | `move` | 目前只实现 `move`（主机侧把 (v, ω) 反解成 0x02 MOVE） |
| `expected_build_id` | `20260A02` | 不符则拒绝一切运动命令；空串=不校验 |
| `max_base_rpm` | 200 | 固件对 MOVE 的上限 |
| `command_period` | 0.1 | 运动期间重发 MOVE 的周期（**不是心跳**） |
| `min_command_interval` | 0.05 | 命令最小间隔，防 ACK 单槽撞车 |
| `reconnect_period` | 0.1 | ★ **必须明显小于 300 ms**，否则断线必锁死底盘 |
| `cmd_vel_timeout` | 0.5 | `/cmd_vel` 超时 → 下发 STOP |
| `wheel_separation_scale` | 1.0 | ★ 标定值，**不要抄仿真的 1.36** |
| `max_linear_vel` / `max_angular_vel` | 0.20 / 1.0 | |
| `allow_run_limit` | false | 真实串口上的安全闸：3.0 s 后会锁死，确认在台架上验证才置 true |
| `dry_run` | false | 只算不发 |

### 8.4 失败要吵

以下情况都会打 ERROR 并写进 latched 诊断，绝不静默：

* 串口打不开 / 写失败 / 重连
* 位流 `build_id` 不符
* 底盘锁死（含故障码的中文含义与**恢复方法**）
* `(v, ω)` 无法表达、被投影
* 内轮目标低于滑行下限（≈7 RPM）
* 0.5 s 没收到遥测

### 8.5 退出行为

`destroy_node` 会发一条 **DISARM**（而不是"STOP + DISARM 连发"）并等它真的发出去。

⚠️ 早先的版本为了"停得更利索"，给发送函数传了一个
`time.monotonic() + 10.0` 去绕过命令限速，结果两条被接受的命令挤进同一个 ACK 槽
→ `tx_overflow` → **底盘永久锁死**。用户只是按了一下 Ctrl-C。
现在限速的判据在发送函数**内部**取时钟，调用方无法绕过 ——
这类自伤从结构上就不可能再发生。

---

## 9. 没有硬件也能验证：虚拟 ACG720

`acg720_simulator.py` 把 `acg720_firmware.py`（**按 RTL 逐条复刻的固件行为模型**）
接到一个真实 PTY 上：

```
虚拟车（master fd）  ←── 内核里连着 ──→  /dev/pts/N（驱动节点打开）
```

两边走**真实的 PTY、真实的 pyserial 配置、真实的字节流**，所以能验证：

✅ 帧编解码 / CRC / 流式重同步（走真字节流与真分帧逻辑）
✅ 驱动的串口打开、读写、断线重连
✅ `/cmd_vel` → MOVE → `/odom` + TF 全链路，含命令超时停车
✅ 运动学符号与尺度（往哪边转、scale 怎么影响转角）
✅ 真固件会咬人的行为：非法帧锁死、ACK 单槽、3.0 s 上限、故障自锁、HEARTBEAT 刹停

❌ **不验证**：电气、真实电机响应、真实编码器、轮胎摩擦、EMI。
这些只有上车才知道，虚拟车不去假装。

### 9.1 用法

```bash
# 自检（不连 ROS，10 项断言，含"非法帧锁死"和"3.0 s 上限"）
python3 src/my_robot_description/src/acg720_simulator.py --dry-run

# 只起虚拟车，打印 /dev/pts/N，自己把驱动指过去
ros2 run my_robot_description acg720_simulator --no-driver

# 一条命令跑通全链路
ros2 run my_robot_description acg720_simulator
```

⚠️ 模型里**所有默认值都是固件真值**（3.0 s 就是 3.0 s）。
`--run-limit-s` 之类的开关只给测试用；在这里放宽等于把要验证的那条限制关掉。

---

## 10. 测试覆盖了什么

| 文件 | 类型 | 覆盖 |
|---|---|---|
| `test_protocol_v1.py` | 纯函数 pytest（68 条） | 真协议编解码、CRC、SEQ、遥测解码；**含 4 条与厂商实现的活体交叉核对** |
| `test_firmware_model.py` | 纯函数 pytest（117 条） | 固件行为：帧层拒绝、语义越界、SEQ、状态机、3.0 s 上限、ACK 溢出、位置环、被控对象 |
| `test_protocol_and_kinematics.py` | 纯函数 pytest（33 条） | 轮位 ↔ 左右、几何换算、MOVE 反解、位置计数口径、位姿积分 |
| `test_hil_end_to_end.py` | 脚本式（59 项） | 真驱动 + 真 PTY + 固件模型：握手、直行、旋转、里程计尺度、超时停车、**不发心跳**、运行上限、超限拦截、SEQ 对齐、短/长掉线、位流校验、位置遥测 |
| `test_bringup_smoke.py` | 脚本式（真 `ros2 launch`） | install 产物、节点/话题/TF、位流校验、状态机使能、自动停车 |

```bash
pytest src/my_robot_description/test/ -q                     # 231 条
python3 src/my_robot_description/test/test_hil_end_to_end.py  # 59 项
python3 src/my_robot_description/test/test_bringup_smoke.py   # 真 launch
```

**诚实说明**：HIL 与冒烟测试的**两端都是我们的代码**（固件模型 + 驱动）。
它们证明的是"这套代码在目标平台上行为正确"，**不证明**"它说的话 FPGA 听得懂"。
后者的证据是 `test_protocol_v1.py` 里与**厂商 `chassis_control.py`** 的逐字节交叉核对，
以及本文档里逐条标注的 `文件:行号`。

---

## 11. 怎么跑起来

### 真机（第一轮：架空车轮）

```bash
ros2 launch my_robot_description robot.launch.py nav:=false rviz:=false
# 另开终端遥控；看 /odom 方向、看急停管不管用
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

⚠️ 真实串口上会用到连续运动，所以启动前确认台架安全。
不加下面这个参数时驱动会**拒绝启动**，并提示"3.0 s 后会锁死底盘" ——
这是刻意的，这条限制不确认就等于让车随时可能锁死：

```bash
ros2 launch my_robot_description robot.launch.py nav:=false allow_run_limit:=true
```

### 没有硬件时

```bash
# 终端 1：虚拟车，打印 /dev/pts/N
ros2 run my_robot_description acg720_simulator --no-driver
# 终端 2：把真机 bringup 指到那个 PTY
ros2 launch my_robot_description robot.launch.py port:=/dev/pts/N nav:=false
```

### 参数怎么量、标完填哪

见 [`real_robot_params.md`](real_robot_params.md)。
FPGA 侧需要改什么见 [`fpga_change_request.md`](fpga_change_request.md)。
