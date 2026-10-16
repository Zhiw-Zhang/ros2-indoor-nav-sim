# FPGA 变更需求（ACG720）

> **给谁看**：电机/FPGA 负责人。
> **写它的原因**：上位机侧（ROS 2 导航）能做的已经做完了。要继续往"连续导航"
> 走，剩下的**全部**障碍都在固件里，而且**只有一处是关键**。
> 本文档给出：现象、RTL 证据（`文件:行号`）、最小改动建议、改完怎么验、
> 以及**不建议**的做法。
>
> 上位机侧对固件行为的复刻在 `src/acg720_firmware.py`，本文档里每一条结论
> 都可以在**没有硬件**的情况下复现（见第 5 节）。

---

## 1. 结论速览

| 优先级 | 变更 | 一句话理由 | 不改的后果 |
|---|---|---|---|
| **P0** | 重新设计 `RUN_LIMIT`（3.0 s 连续运动上限）策略 | 它是**唯一**挡住连续导航的东西；而 300 ms 命令超时**已经**是防跑飞的 deadman，3 s 上限是冗余的 | 连续 `MOVE` 闭环**永远做不了**；只能改用位置段分段导航兜底（见 7.3） |
| P1 | 允许内侧轮**反转**（或给内/外轮独立符号） | 现在内侧轮只能"按比例缩小" | 转弯半径 < 半轮距的急弯、以及**倒车转弯**不可达 |
| P2 | 新增 `0x07 VELOCITY` | **建议不做**（见第 4 节）：现有 `0x02 MOVE` 已能表达绝大多数 (v, ω)，而且 `INTERFACE.md` 的草案会被现有解析器直接拒绝 | —— |
| 文档 | 统一 `INTERFACE.md` 的 LEN 上限（48 vs 实际 8） | 草案按现状发出去会**锁死底盘** | 照草案实现的人一定会踩 |

**一句话**：只要做 P0 这一处，上位机侧**已经写好**的连续控制就能立刻用起来
（`(v, ω)` → `0x02 MOVE` 的反解已经实现并测试过）。
P1 是"能做多灵活"的问题，不是"能不能做"的问题。

> **但有一个重要例外**：`POSITION`（`0x05`）模式**不受 3.0 s 上限约束**
> —— 判据里的 `!position_active`（`:57-58`）。所以在上位机侧把长距离拆成
> 一串位置段，**不改固件也能实现连续导航**。
> 这条路径成立的原因非常微妙（靠一个分支的先后顺序，见 **7.3**），
> 而且它**不是**"走走停停凑合"：每段结束时底盘只回到 `ARMED_STOP`，
> 不锁死、不需要人工复位，段与段之间可以直接接着发。
> 它是 P0 落地之前的正经替代方案。

---

## 2. P0：`RUN_LIMIT` 策略

### 2.1 现状（RTL 原文）

```verilog
// chassis_safety_fsm.v:10-11
parameter [31:0] COMMAND_TIMEOUT_CYCLES = 32'd15_000_000,   // 300 ms
parameter [31:0] RUN_LIMIT_CYCLES       = 32'd150_000_000   // 3.0 s

// :56-58
assign timeout_reached   = (command_age >= COMMAND_TIMEOUT_CYCLES - 32'd1);
assign run_limit_reached = !position_active &&
                           (run_age >= RUN_LIMIT_CYCLES - 32'd1);

// :72-77  ★ run_age 只在**离开 RUNNING** 时清零
always @(posedge clk or negedge reset_n) begin
    if (!reset_n || (state != STATE_RUNNING)) run_age <= 32'd0;
    else if (!run_limit_reached)              run_age <= run_age + 32'd1;
end

// :92-97
end else if ((state == STATE_RUNNING) && internal_stop) begin
    state <= STATE_ARMED_STOP;  fault_reason <= FAULT_NONE;
end else if ((state == STATE_RUNNING) && run_limit_reached) begin
    state <= STATE_FAULT;       fault_reason <= FAULT_RUN_LIMIT;
end

// :88  ★ FAULT 自锁
end else if (state == STATE_FAULT) begin
    state <= STATE_FAULT;
end
```

### 2.2 现象

* 任何**连续**运动（`0x02 MOVE` 或 `0x04 OPEN_LOOP`）满 3.0 秒 →
  进 `FAULT_RUN_LIMIT`，电机使能切断，**只能断电或"松开 S0 再按 S4"复位**。
* **重发命令不能续期**：`run_age` 只在离开 RUNNING 时清零，所以
  主机以 10 Hz、20 Hz 甚至 100 Hz 重发 MOVE，也一样在 3.0 秒触顶。
  （这一点容易被误判成"发得不够勤"，实测结论是发多勤都没用。）
* `0x05 POSITION` **不受**这条限制（`!position_active`），
  所以位置动作可以跑十几秒 —— 这也是今天唯一能真正驱动真车的运动原语。

### 2.3 为什么它挡住了连续导航

Nav2 这类导航栈的输出是**持续变化的 (v, ω) 速度流**（典型 20 Hz），
一次"走到目标点"通常要 5～30 秒。

* 用 `0x02 MOVE`：3.0 秒后锁死，导航走不完，而且**锁死之后需要人工复位**。
* 用 `0x05 POSITION`：只能"走直线段 → 停稳 → 原地转 → 再走"，
  路径是折线、每次都停一下，而且每次动作都要等 `SETTLE_TICKS` 判稳。
  这能凑合做"分段导航"，但速度慢、乘坐体验差，且**无法跟踪动态障碍**。

### 2.4 建议的改法

**首选（推荐）**：删掉 `run_limit_reached` 这一条分支（或把
`RUN_LIMIT_CYCLES` 提到明显用不到的量级），**只保留 300 ms 命令超时**。

理由：`command_age` 已经提供了等价的、而且**更灵敏**的失控保护 ——

```verilog
// chassis_safety_fsm.v:118-121
end else if (armed && timeout_reached && !command_valid) begin
    state <= STATE_FAULT;  fault_reason <= FAULT_TIMEOUT;
end
```

主机只要停止发命令 300 ms（进程崩了、串口掉了、上位机断电），
底盘立刻进 `FAULT_TIMEOUT` 并切断使能。这比"连续跑满 3 秒才停"**快 10 倍**。
换句话说：**3 秒上限保护的那件事，300 ms 超时已经保护了，而且保护得更好。**

**备选**：如果出于台架/演示的考虑想保留，可以只在 `OPEN_LOOP` 模式下生效
（那种模式本来就是台架标定用的），`MOVE`/`POSITION` 不受限：

```verilog
assign run_limit_reached = active_open_loop && !position_active &&
                           (run_age >= RUN_LIMIT_CYCLES - 32'd1);
```

**不推荐**：把上限从 3 s 改大（比如 30 s）。那只是把"走到一半锁死"推迟到
"走到一半锁死"，问题不变，还削弱了原本的安全裕度。

> ⚠️ **无论采用哪种改法，都千万不要动** `internal_stop` 与 `run_limit_reached`
> 两个分支的**先后顺序**，也不要顺手把 `!position_active` 去掉。
> 那是"位置段能跑超过 3 s"的**唯一**依据，理由见 7.3 —— 很容易在重构时
> 无意破坏，而破坏后的现象极难定位。

### 2.5 改动后的验证方法

1. **无硬件**：本仓库的虚拟底盘**已经复刻**了这条限制。把
   `FirmwareParams.run_limit_s` 改成 `inf`（或按 2.4 的改法改掉判据），
   然后跑：

   ```bash
   python3 src/my_robot_description/test/test_hil_end_to_end.py
   ```

   改动前 `[8] 3.0 s 连续运动上限` 那一节会因为**看不到 FAULT_RUN_LIMIT** 而失败 ——
   这正是"改成功了"的信号。相应地，
   `test_firmware_model.py::test_run_limit_latches_even_with_continuous_move_stream`
   与 `test_run_age_is_not_reset_by_resending_move` 要按新策略更新。

2. **有硬件**：同一段 `/cmd_vel` 序列（例如 0.1 m/s 直行 10 秒）：

   | | 改动前 | 改动后 |
   |---|---|---|
   | 结果 | 3.0 s 后停住并锁死，`0x81` 报 `state=3 fault=5` | 走满 10 s，`/odom` 位移 ≈1 m |
   | 恢复 | 必须断电/按 S4 | —— |

   注意：**不要**用"周期 STOP 再 MOVE"去凑，那会把 3 秒上限的语义彻底绕掉，
   而且车会每 3 秒急停一次（见第 6 节）。

### 2.6 风险与安全论证

| 风险 | 论证 |
|---|---|
| 去掉 3 s 上限后，主机崩了车会不会一直跑？ | 不会。300 ms 命令超时（`:118-121`）会进 `FAULT_TIMEOUT` 并切断使能。**比 3 s 更快。** |
| 主机发命令但车撞墙呢？ | 那属于导航/避障层的问题（上位机已有 `collision_monitor`），不是"连续跑多久"的问题；3 s 上限对撞墙没有任何帮助。 |
| 传感器/编码器坏了导致车一直冲？ | 现有机制：`motor_fault`（电机通道故障）、`FAULT_ESTOP`（硬件急停 S0）。这些与 `RUN_LIMIT` 无关，不受影响。 |
| 会不会有人依赖 3 s 上限做"演示模式的自动停"？ | 若有此需求，建议**用主机侧**实现（发 3 秒就发 STOP），而不是把它锁死在固件里 —— 固件层一锁死就需要人工复位，代价太大。 |

---

## 3. P1：允许内侧轮反转

### 3.1 现状

```verilog
// chassis_motion.v:32-36
assign target_valid = (base_rpm <= 16'd200) && (motion_mode <= MODE_ROTATE_RIGHT);
assign inner_product = base_rpm * turn_inner_ratio_q8;
assign outer_rpm = $signed(base_rpm);
assign inner_rpm = $signed(inner_product[23:8]);      // ★ 无符号右移，恒 ≥ 0
```

mode 3/4（左转/右转）里，外侧轮 = `base_rpm`，内侧轮 = `base_rpm × ratio/256`，
所以**内侧轮只能比外侧慢，不能反转、不能独立赋值**。

### 3.2 影响（上位机侧的实测结论）

好消息：它**能**表达绝大多数 `(v, ω)`。`(base_rpm, ratio)` 是两个自由参数，
正好解两个未知数；推导见 [`acg720_protocol.md` 第 6.2 节](acg720_protocol.md)。
`ratio = 0`（内轮停转、绕内轮转）对应**最小转弯半径 = 有效轮距/2 ≈ 0.10 m**。

表达不出来的只有两处：

| 情况 | 现在会怎样 | 影响 |
|---|---|---|
| 转弯半径 `< L/2`（急弯、原地掉头的小半径） | 主机**投影**成原地转（丢掉 v） | 跟踪误差，但不会失控；导航层可用 `RotateToGoal` 之类的行为绕开 |
| **倒车转弯**（v<0 且 ω≠0） | 主机**投影**成直退（丢掉 ω） | 倒车入库/狭窄处脱困能力受限 |

也就是说：**P1 不做也能跑**，只是"倒车脱困"和"急弯"这两类动作要绕开。

### 3.3 建议的改法

把 `inner_rpm` 改成有符号、并给主机**独立符号**的能力。最小侵入的做法是
复用现有的 `turn_inner_ratio_q8` 字段的**最高位当符号位**
（`0..255` 分解为 `sign + 0..127` 的比例），这样**帧格式和 LEN 都不用变**，
老的主机代码（只用 `0..127`）行为不变。

（另一种做法是新增一条命令，但那要动 LEN 上限和语义校验，见第 4 节 —— 不划算。）

### 3.4 验证方法

改动后，上位机侧 `drive_kinematics.twist_to_move()` 里那两条投影告警会消失，
`test_protocol_and_kinematics.py::test_sharper_than_half_track_is_projected_to_spin`
与 `test_reverse_arc_is_projected_to_straight_backward_preserving_v`
需要按新能力更新（现在它们钉的是"不可达"这个事实）。

---

## 4. P2：新增 `0x07 VELOCITY` —— 建议**不做**

`navigation/docs/INTERFACE.md` 提议新增：

* `0x07 VELOCITY`，`LEN = 10`（i32 v_mm_s + i32 ω_mdeg_s + u16 timeout_ms）
* `0x85 NAV_TELEMETRY`，`LEN = 36`

**两条都被现有解析器直接拒绝：**

```verilog
// chassis_command_rx.v:208
if (rx_data > 8'd8) begin
    state <= ST_HEADER_0;
    protocol_fault <= 1'b1;      // ★ 锁死底盘
end
```

* 固件接受的 **LEN 上限是 8**，而草案是 10 → 发出去就是 `protocol_fault`
  → **永久锁死**。所以那份草案**不能按现状烧录/发送**（文档自己也这么写了，
  这点值得肯定）。
* 即使把 LEN 压到 8，`0x07` 也**未分配、未实现**（`command_kind` 只有 3 位，
  0x08 及以上永远过不了语义校验），必须先实现才有意义。

**而它想解决的问题已经不存在了**：`0x07` 的动机是"MOVE 表达不了 Nav2 的 (v, ω)"，
但第 3.2 节的推导说明 MOVE 能表达 `R ≥ L/2` 的**全部** (v, ω)。
真正挡住连续导航的是 P0，不是"缺少速度命令"。

**结论**：先做 P0；P1 视需要；P2 在做完 P0 之前不用考虑。
若将来真要做 P2，务必同时处理 LEN 上限与语义校验。

---

## 5. 这些结论怎么在没有硬件时复现

上位机侧有一份**按 RTL 逐条复刻**的固件行为模型
（`src/acg720_firmware.py`，117 条单测钉住），加一个把模型接到真 PTY 上的
虚拟底盘（`src/acg720_simulator.py`）。开箱即可复现本文档的全部现象：

```bash
# ① 自检：10 项，含"非法帧锁死"与"3.0 s 上限"
python3 src/my_robot_description/src/acg720_simulator.py --dry-run

# ② 117 条固件行为单测（含 run_age 不被重发清零、HEARTBEAT 会刹停等）
pytest src/my_robot_description/test/test_firmware_model.py -q

# ③ 59 项端到端（真驱动 + 真 PTY + 固件模型），含
#    "3.0 s 后锁死并如实上报" 与 "断线 >300 ms 必锁死"
python3 src/my_robot_description/test/test_hil_end_to_end.py
```

其中直接对应本文档的三条测试：

| 断言 | 对应本文档 |
|---|---|
| `test_run_limit_latches_even_with_continuous_move_stream` | 2.2：10 Hz 重发也躲不过 3 s |
| `test_run_age_is_not_reset_by_resending_move` | 2.2：重发不清零 `run_age` |
| `test_stop_then_move_restarts_run_age_which_is_why_cycling_is_forbidden` | 6：周期 STOP 之所以"有效"的原理（也正是它被禁止的原因） |

---

## 6. **不建议**的做法

| 做法 | 为什么不行 |
|---|---|
| 主机侧"周期 STOP + 重新 MOVE"绕过 3 s 上限 | 交接包 `INTERFACE.md` 明令禁止；而且车会每 3 秒急停一次，动态障碍下更危险；还把"该改固件"这件事掩盖成"主机在正常工作" |
| 主机侧"周期性重发 MOVE"想续期 | **物理上无效**：`run_age` 只在离开 RUNNING 时清零，重发不清零（有专门的测试） |
| 把 `RUN_LIMIT_CYCLES` 从 3 s 改大到 30 s | 只是把问题推远；反而削弱原本的安全裕度 |
| 按 `INTERFACE.md` 草案直接发 `0x07`/`0x85` | LEN 超 8 → `protocol_fault` → **永久锁死底盘** |
| 用"改阈值"代替"改策略" | 见上一条：需要的是**判断依据**的变更，不是数字的变更 |

---

## 7. 顺带发现的三处（不是变更请求，是提醒）

1. **`chassis_telemetry_tx.v` 是遗留文件，`top.v` 实例化的是
   `chassis_telemetry_imu_tx.v`**（`:342`）。两份文件长得像，
   照着前者实现解码会得到错误的帧长/字段。建议在文件头加一行注释说明它已废弃，
   或直接移出综合目录。
2. **`FAULT_PROTOCOL` 里"ARMED_STOP 下带非零目标的 ARM"这条分支不可达。**
   `ARM` 要求 `LEN=0`，而 `chassis_command_rx.v:273-279` 对非
   MOVE/OPEN_LOOP/POSITION 的命令会把 `motion_mode`/`base_rpm` 清零，
   所以 `command_stop_target` 恒为真。功能上无害（只是死代码），
   但**别把它当成一条可用的保护**：真正的保护是"ARM 只在 DISARMED 下被接受"。
   上位机侧我们已经按"ARM 只发一次、绝不重发"实现。
3. ★★ **`chassis_safety_fsm.v` 里 `internal_stop` 与 `run_limit_reached`
   这两个分支的先后顺序是"承重"的 —— 改 P0 时最容易踩坏的就是它。**

   先看判据（`:57-58`）：

   ```verilog
   assign run_limit_reached = !position_active &&
                              (run_age >= RUN_LIMIT_CYCLES - 32'd1);
   ```

   再看 `run_age` 的累加条件（`:74-77`）——**它只在 `state != RUNNING` 时清零**：

   ```verilog
   always @(posedge clk or negedge reset_n) begin
       if (!reset_n || (state != STATE_RUNNING))
           run_age <= 32'd0;
       else if (!run_limit_reached)
           run_age <= run_age + 32'd1;
   end
   ```

   把这两段合起来看，就能推出一个反直觉的结论：**位置移动期间 `run_age`
   照常累加，一路加过 3.0 s 也不会停**（因为 `run_limit_reached` 被
   `!position_active` 压成了假，累加分支一直成立）。也就是说，
   **一个超过 3.0 s 的位置段走完的那一刻，`run_age` 早就爆表了**，
   `run_limit_reached` 会在同一拍立刻变真。

   它之所以**没有**跳 `FAULT_RUN_LIMIT`，唯一的原因是判断的先后顺序：

   ```verilog
   end else if ((state == STATE_RUNNING) && internal_stop) begin      // :91 先判
       state        <= STATE_ARMED_STOP;
       fault_reason <= FAULT_NONE;
   end else if ((state == STATE_RUNNING) && run_limit_reached) begin  // :95 后判
       state        <= STATE_FAULT;
       fault_reason <= FAULT_RUN_LIMIT;
   end
   ```

   而位置环的每一条结束路径（正常走完 / 超时 / 堵转 / 被拒）都把
   `busy <= 1'b0` 和 `internal_stop_pulse <= 1'b1` 写在**同一个分支**里：

   | 结束原因 | 出处 |
   |---|---|
   | 起步参数被拒 | `chassis_position_controller.v:199-205` |
   | 超时 | `:222-225` |
   | 堵转 | `:236-239` |
   | 正常走完（SETTLE 结束） | `:254-258` |

   两者都是非阻塞赋值，所以在 `position_active` 由 1 变 0 的那一拍，
   `internal_stop` **一定同时**为 1，`:91` 抢在 `:95` 前面把状态带去
   `ARMED_STOP`，`run_age` 随即被清零。**这是在一条极细的缝上走对了。**

   由此有两条硬约束：

   * **别把 `:95` 挪到 `:91` 前面**，也别把 `run_limit_reached` 改成不排除
     `position_active`。一旦改动，**每一个超过 3 s 的位置段都会在到达目标的
     那一瞬间锁死底盘**，现象是"车明明走到位了却报 `RUN_LIMIT`"，
     而且只在长距离时复现 —— 极难定位。
   * 位置环新增任何结束路径时，**必须**同样成对地写
     `busy <= 1'b0` 与 `internal_stop_pulse <= 1'b1`。漏掉 `internal_stop_pulse`
     就等于同时踩中"`run_age` 不清零"和"位流状态机停在 RUNNING"两个坑。

   > 附带说明：正因为这条顺序存在，**上位机用"位置环分段导航"（把长距离拆成
   > 一串 `0x05 POSITION`）在当前位流上就能实现连续导航，不必等 P0 落地**。
   > 这条路径能成立完全是靠上面的时序，所以它同样是"改 RTL 前必须知道"的事。

---

## 8. 一页纸的行动清单

```
[ ] P0  决定 RUN_LIMIT 怎么改（推荐：删掉判据，只留 300 ms 命令超时）
[ ] P0  改 RTL 时**保住** `:91`/`:95` 的分支顺序与 `!position_active`（见 7.3）
[ ] P0  改完跑一遍无硬件验证（第 5 节三条命令），确认"3.0 s 上限"相关的断言按预期翻转
[ ] P0  在台架上跑"同一段 /cmd_vel 跑 10 秒"的前后对比（2.5 节表格）
[ ] 文档 把 INTERFACE.md 的 LEN 上限从 48 改成 8，并标注 0x07/0x85 尚未分配
[ ] P1  （可选）给内侧轮加符号位
[ ] P2  （不需要）0x07 VELOCITY
[ ] 替代方案（不改固件）：上位机做"位置段分段导航"，见 7.3 末尾
```
