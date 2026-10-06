# 功能验收：自主移动 / 障碍物避让 / 路径点导航

对应三条需求。**每条都给了测试方法和实测数据**，包括没通过的。

两个场地，两轮验收：

| 场地 | 说明 |
|---|---|
| `worlds/rooms.sdf` | 两间房 + 走廊，无障碍物。第一轮，验基础能力 |
| `worlds/rooms_obstacles.sdf` | 同一个房间布局 **+ 5 个静态障碍物**，门AB 被封死。第二轮，三个功能重测一遍 |

| # | 要求 | 结论 | 一句话 |
|---|---|---|---|
| ① | 起点到终点自主移动 | ✅ **达标** | 空场地 6/6、4/4、4/4；带障碍物场地绕行 2.91× 到达，零接触 |
| ② | 障碍物避让 | ✅ **达标**（静态障碍物） | 5 个障碍物（含 6 cm 细杆）全部绕开、三次测试**零接触**，而且不是靠兜底刹车 |
| ③ | 路径点导航 | ✅ **达标** | 带障碍物场地 5/5 一次调用；路点不可达时按配置跳过并报出准确错误码 |

> 场地②里发现一个**可复现的窄通道卡死**（斜墙东端 0.726 m 那条缝），
> 见本文最后的"发现的问题"。它不是功能缺失，是 `inflation_radius` 和通道宽度
> 配不上，但确实会让 ③ 在某个起点上失败。

复现用的工具都在 `scripts/` 里，见 `docs/tools.md`。

---

## ① 起点到终点自主移动

**测试方法**：下发 `NavigateToPose`，路线故意串起三个门洞、两个房间和一条走廊，
终点误差按 **Gazebo 真值位姿**算，不是看 RViz 像不像。

| 场景 | 结果 | 终点误差 | 出处 |
|---|---|---|---|
| 已有地图 + AMCL，6 个目标 | **6/6 SUCCEEDED** | 真值均值 **16.0 cm**，最大 30.7 cm | `docs/nav2.md` |
| 边建图边导航（`slam:=true`），4 个目标 | **4/4 SUCCEEDED** | 10.1 ~ 13.9 cm | `docs/nav2.md` |
| 路径点导航一次调用 4 个路点 | **4/4 到达** | —— | 本文档 ③ |

定位误差（`/amcl_pose` 对同刻真值，212 样本）：位置均值 **6.48 cm**、
95 分位 12.71 cm、航向 1.43°。全程真值轨迹离墙最近 **35.3 cm**，没有一次擦碰。

**前提（属于操作要求，不是功能缺失）**：用已有地图 + AMCL 启动时，
`set_initial_pose: true` 假设"车就停在建图时的起点"。车放在别处，
必须用 RViz 的 **2D Pose Estimate** 告诉它位置，否则定位是错的。
（这条在测试过程中亲身踩到：只重启 Nav2 而没重设初值，帧偏了 10 cm，导航直接失败。）
用 `slam:=true` 则不需要，它以自身起点为原点现场建图。

---

## ② 障碍物避让（第一轮：运行时生成的**未知**障碍物）

> 这一轮测的是"地图上没有、雷达临时看到"的障碍物，**其中 T2（车前突然出现）
> 没有通过**。第二轮改用 `rooms_obstacles.sdf` 里**图上已知**的静态障碍物重测，
> 全部通过 —— 见下面"第二轮"一节。

**测试方法**：障碍物**不进地图**，用 Gazebo 的 `/world/default/create` 服务在
**运行时**生成 —— 这正是"地图上没有、雷达突然看到"的场景。
工具 `scripts/obs_test.py` 同步记录机器人位姿、`/cmd_vel` 和
`/collision_monitor_state`，算与障碍物的最小间距。

判读口径：`最小间距` 是机器人中心到障碍物矩形的距离。车体外接圆半径 27.3 cm，
所以 **> 27.3 cm 就是确定没碰上**。

### 结果

| 测试 | 场景 | 结果 | 最小间距 | 代价图标记 | 绕行横向偏移 | collision_monitor |
|---|---|---|---|---|---|---|
| T0 | 无障碍物（基线） | ✅ SUCCEEDED 24.4 s | —— | —— | 0 cm | 未介入 |
| **T1** | 静态未知障碍 0.4×0.4×0.6 m，一开始就挡在路径正中 | ✅ SUCCEEDED 14.9 s | **33.5 cm** | **26 个致命格** | **70.3 cm** | 未介入（不需要） |
| **T5** | 细杆 0.05×0.05×0.6 m，同上 | ✅ SUCCEEDED 13.1 s | **39.2 cm** | **2 个致命格** | 37.4 cm | 未介入（不需要） |
| **T2** | 车开到一半，障碍物在车前 **0.7 m 突然出现** | ❌ **150 s 未完成** | 29.3 cm | 6 个致命格 | 11.0 cm | **APPROACH 立刻介入** |

**T1 / T5 说明核心能力是有的**：地图上不存在的障碍物被雷达标进代价图
（`/global_costmap/costmap` 上读到致命格），全局规划重新规划，
车绕开 37 ~ 70 cm 通过，全程没碰、没停。

> 代价读数的坑：这条话题上的代价是**缩放到 0~100** 发布的，不是常见的 0~255。
> 判据是实测出来的——地图里**已知的墙**在这条话题上读到的就是 100。

### T2：没通过，以及为什么

T2 是"车前 0.7 m 突然出现障碍物"。**安全刹停是有效的**，
`collision_monitor` 在 0.05 s 内就进入 APPROACH 并接管，车没撞上去。
但**车随后卡死了 150 秒没能脱困**，日志是这样一个循环：

```
collision_monitor: Robot to approach for 1.200000 seconds away from collision
controller_server: Passing new path to controller      ← 全局规划一直在给新路径
controller_server: Failed to make progress             ← 但车挪不动 → 判超时
  → behavior_server: Running spin   → spin failed      → Running wait → 重试
  → behavior_server: Running backup → backup failed    → 重试
  → 再次 Failed to make progress ...                    （循环 11 次以上）
```

**根因**（有日志和数据支撑，不是猜的）：

1. `collision_monitor` 用的是 `action_type: "approach"`——它**调节接近速度**，
   不保证不接触。车被压着慢慢挪，一直挪到车前脸距障碍物约 0.1 m。
2. 此时车的 footprint 已经压进障碍物的**内切膨胀区**（`inflation_radius: 0.35`，
   内切半径 0.185 m），在代价图上就是"车已经在碰撞状态里"。
3. 于是：MPPI 找不到无碰撞轨迹（日志里有一次 `Optimizer fail to compute path`），
   而 `spin` / `backup` 这两个恢复行为**自己也要做碰撞检查**，
   当前状态已经"在碰撞中"，它们也走不完，全部失败。
4. 恢复行为失败 → 重试同一条路 → 再次失败 → 无限循环。

**已做的修正**：`progress_checker.required_movement_radius` 从 `0.5` 收到 `0.2`。
理由是 0.5 m 要求一台 **车长只有 0.40 m** 的车在 15 s 内移动超过自身长度才算"有进展"，
本身不合理。实测效果：**最小间距从 19.9 cm 改善到 29.3 cm**，
从"可能擦碰"变成"确定没碰上"。

**但这个修正没有解开死锁**——因为瓶颈不在进度阈值，而在上面第 3 步：
车一旦进入内切膨胀区，恢复行为自己也被碰撞检查挡住。
要真正修掉，得从"别让它陷进去"或"让恢复行为能在已碰撞状态下工作"入手，
比如把 `collision_monitor` 的 `action_type` 从 `approach` 换成 `stop`
（更早硬停，停在内切区之外），或者调大 `time_before_collision`。
**这些改动会影响所有导航行为，需要单独验证，本次没有做。**

### 已知的残余风险（明确没测）

| 项 | 为什么值得测 |
|---|---|
| 门洞里的障碍物（T3） | 0.9 m 的门只剩约 0.43 m 可通行带，一个 0.4 m 箱子就堵死。是"绕过"还是"判定不可达"？没测 |
| 移动障碍物（T4） | 横穿车的路径时 MPPI 能不能躲。没测 |
| 细杆 + 近距离突现 | `collision_monitor` 的 `min_points: 6` 意味着多边形里少于 6 个雷达点就不判碰撞。一根 5 cm 杆在 2 m 外只有 1~2 个点，**兜底可能被绕过**。T5 之所以没事是因为代价图/MPPI 先把它躲开了；如果它突然出现在极近处，兜底是空的。**这是我最担心的一条，没测** |

---

## ③ 路径点导航

**两种做法都可用**：

* `nav2_waypoint_follower`（标准做法）——一次 `FollowWaypoints` action 下发全部路点
* 连续发 `NavigateToPose`——功能等价，每段走一次完整行为树

测试用第一种：`nav_goals.py --mode follow_waypoints`。

| 测试 | 路点 | 结果 |
|---|---|---|
| ③-A | 4 个，串起三个门（房间A→门A→走廊东端→门B→房间B） | **4/4 到达**，一次调用，68.7 s，整体 status=4 |
| ③-B | 4 个，其中第 2 个是 (10, 10)（**在地图外，不可达**） | **整体 status=4**，`missed_waypoints = [index=1, error_code=204 GOAL_OUTSIDE_MAP]`，**跳过它继续走完第 3、4 个** |

③-B 验证了两件事：
1. `waypoint_follower.stop_on_failure: false` 确实生效——单个路点失败不会中断整条路线；
2. 失败原因诊断准确：`204 = GOAL_OUTSIDE_MAP`，不是随便报个错。

> `MissedWaypoint.error_code` 用的是 `nav2_msgs/ComputePathToPose` 那套规划错误码
> （200 UNKNOWN / 202 TF_ERROR / 204 GOAL_OUTSIDE_MAP / 208 NO_VALID_PATH …）。

---

## 第二轮：带障碍物的新世界 `worlds/rooms_obstacles.sdf`

上面的 ①②③ 都是在 `rooms.sdf`（空房间 + 走廊）里测的。这一节换一个**自带 5 个
静态障碍物**的新世界重测一遍 —— 这才是"指定场景"该有的样子。
**本轮不测"障碍物突然出现"**，只测地图上已知的静态障碍物。

### 场地

`rooms_obstacles.sdf` = `rooms.sdf` 的墙体 + 5 个障碍物。全部 `static`、高 1.0 m
（雷达装在 z≈0.22，一定打得到）：

| 名称 | 位置 | 尺寸 | 物理剩余通道 | 作用 |
|---|---|---|---|---|
| `obs_corridor` | (0.0, 1.55) | 1.20×0.40 | 南 0.275 / 北 **0.675** | 走廊中段横躺，只能贴北墙过 |
| `obs_roomA_block` | (-1.6, -1.0) | 0.80×0.80 | 四周 ≥1.0 | 房间A 正中，挡住出生点→门A 的直线 |
| `obs_door_ab` | (1.0, -0.6) | 0.30×0.30 | 两侧各 **0.300** | 正塞在门AB 中间 → 这个门对车等于封死 |
| `obs_roomB_slant` | (3.4, -1.4) | 1.60×0.35，**yaw 25°** | —— | 斜墙，考验 SLAM 斜边 + 非轴对齐避障 |
| `obs_pole` | (2.2, 0.4) | **0.06×0.06** | —— | 细杆，看能不能进图、能不能被绕开 |

关键设计：机器人 footprint 0.40×0.37、内切半径 0.185 m，**任何小于 0.37 m 的缝
对它都等于封死**。`obs_door_ab` 就是这么用的 —— 它把"房间A ↔ 房间B"的最短直线
路径彻底切断，想去房间B 必须绕：房间A → 门A → 走廊（贴北墙过 `obs_corridor`）
→ 门B → 房间B。

### 地图

全自动建的（`slam:=true` + 17 个路点的覆盖路线，**没有人工遥控**），
17/17 路点到达、303 s，产物是 `maps/rooms_obstacles.yaml`。

![地图与真实几何对照](rooms_obstacles_map.png)

| 指标 | 数值 |
|---|---|
| 尺寸 | 200×103 格 @ 0.05 m = 10.00×5.15 m（world 是 10×5 m） |
| 占用格到最近真实几何 均值 | **1.62 cm**（≤5 cm 86.37%，≤10 cm 98.69%，≤15 cm 100%） |
| 幻影块 | **0** |
| 可见墙面覆盖率 | **96.27%** |
| 5 个障碍物是否进图 | **5/5**，最近占用格距离全是 **0.0 cm** |

连 6 cm 见方的 `obs_pole` 都进了图（`mapeval.py` 的"障碍物进图情况"一节直接可查）。

### ① 起点到终点

| 起点 | 终点 | 结果 | 用时 | 实际路径 | 直线 | 绕行 |
|---|---|---|---|---|---|---|
| (0, 0) 出生点 | (4.2, -1.9) 房间B东南角 | ✅ SUCCEEDED | 60.1 s | **13.42 m** | 4.61 m | **2.91×** |
| (4.2, -1.9) | (0, 0) 出生点 | ✅ SUCCEEDED | 64.0 s | **13.31 m** | 4.64 m | **2.87×** |
| (0, 0) | (1.5, 0.3) 房间B里 | ✅ SUCCEEDED | 53.3 s | **12.05 m** | 1.66 m | **7.25×** |

第三行最能说明问题：目标直线距离只有 **1.66 m**，但门AB 被封死，实际必须绕
**12.05 m**（7.25 倍）。三趟的终点误差 13.0 / 20.4 / 6.6 cm。

### ② 障碍物避让

判定用**仿真真值位姿** + OBB-OBB 分离轴测试（车体和障碍物都按**真实朝向**），
不是看 RViz 像不像：

| 测试 | 结果 | 到 5 个障碍的最小间距 | 到墙最小间距 | 接触次数 |
|---|---|---|---|---|
| 出生点 → 房间B东南角 | ✅ | 29.8 / 70.9 / 96.2 / 36.6 / 82.1 cm | 30.3 cm | **0** |
| 房间B东南角 → 出生点 | ✅ | 30.2 / 77.4 / 115.1 / 34.0 / 85.5 cm | 31.0 cm | **0** |
| 出生点 → (1.5, 0.3)（贴着细杆走） | ✅ | 28.8 / 85.7 / 69.8 / 144.3 / 44.8 cm | 30.1 cm | **0** |

（间距顺序：`obs_corridor` / `obs_roomA_block` / `obs_door_ab` / `obs_roomB_slant` / `obs_pole`）

三次都**零接触**，而且 `collision_monitor` **一次都没介入过** —— 不是靠兜底
刹停混过去的，是规划器一开始就把路绕开了。最紧的一处是走廊里贴 `obs_corridor`
过，三次分别留了 29.8 / 30.2 / 28.8 cm，很稳定（那条通道物理上只有 0.675 m）。

`obs_pole` 间距 44.8 cm，说明 6 cm 的障碍物进了图之后确实会被绕开。

### ③ 路径点导航

| 测试 | 路点 | 结果 |
|---|---|---|
| ③-A | 5 个：跨门A → 走廊（贴障碍物北侧）→ 门B → 绕过斜墙到房间B东南角 | **5/5 到达**，一次调用，136.4 s，status=4 |
| ③-B | 4 个，第 2 个 (1.0, -0.6) **落在 `obs_door_ab` 里** | 整体 status=4；#2 报 `208 NO_VALID_PATH` 被跳过，#1 报 `105 FAILED_TO_MAKE_PROGRESS`（见下），#3 #4 到达 |

③-B 验证了 `stop_on_failure: false` 确实生效（有路点失败仍把其余走完），
以及错误码诊断准确：算不出路径给 `208`，算出来了走不动给 `105`。

> `MissedWaypoint.error_code` 会带**两套**错误码：100 段来自
> `nav_msgs/action/FollowPath`（走不动），200 段来自
> `nav_msgs/action/ComputePathToPose`（算不出）。一开始只列了 200 段，
> 于是报出 `error_code=105` 时看不懂是什么意思。

### 发现的问题：窄通道会卡死（可复现）

从**房间B 的东南角**（斜墙背后那个口袋）往外走，会**稳定地**卡住：

| 尝试 | 起点 | 终点 | 结果 |
|---|---|---|---|
| 第 1 轮 | 房间B东南角 (4.24, -1.94) | (0, 0) | ❌ 200 s 没走完 |
| 第 1 轮 ③-B #1 | 房间B东南角 (4.31, -1.88) | (-4.0, 0.4) | ❌ `105 FAILED_TO_MAKE_PROGRESS` |
| 第 2 轮 | 房间B东南角 | (0, 0) | ✅ 64.0 s 走完 |
| 第 2 轮 ③-B #1 | 房间B东南角 (4.31, -1.90) | (-4.0, 0.4) | ❌ `105 FAILED_TO_MAKE_PROGRESS` |

日志里的signature完全一致，**起点坐标都停在 `(4.5, -1.3)` 一带**：

```
[ERROR] [planner_server]: Failed to create a plan from potential when a legal
        potential was found. This shouldn't happen.
[WARN]  [planner_server]: GridBased plugin failed to plan from (4.53, -1.32) to
        (1.00, -0.60): "Failed to create plan with tolerance of: 0.250000"
[ERROR] [controller_server]: Failed to make progress          ← 单轮出现 14 次
[WARN]  [behavior_server]: spin failed / backup failed
```

`(4.5, -1.3)` 正是 `obs_roomB_slant` 东端与东墙之间那条 **0.726 m** 的缝。
用 `cost_slice.py` 量它的代价图：

```
   y       x=4.40  4.45  4.50  4.55  4.60  4.65  4.70
 -1.20       99     94    81    71    82    96    99
 -1.25       99     89    78    71    82    96    99
 -1.30       92     82    73    71    82    96    99
 -1.35       82     75    67    71    82    96    99
```

**这条缝里最便宜的格子是 67~71，一半的格子是 99**
（99 = `INSCRIBED_INFLATED_OBSTACLE`，车心在那种格子里车体就已经压到障碍物了）。
成因是 `inflation_radius: 0.35`：0.726 m 的缝两边各铺 0.35 m 膨胀，中间几乎
不剩东西。

于是机器人一旦进去，NavFn **从那个起点出发连一条全局路径都算不出来**，
控制器又动不了，恢复行为也出不来 —— 而旁边明明还有一条**代价全 0** 的宽路
（往西绕过斜墙西端）。规划器之所以会选这条窄的，是因为 NavFn 的代价模型里
一格 71 只相当于自由格的约 2 倍（`neutral_cost=50`、`cost_factor=0.8`），
"短而贵"和"长而便宜"两条路算出来几乎一样。

**换算关系**：这车配 `inflation_radius: 0.35` 时，通道宽度最好 ≥ **1.0 m**；
0.70 m 是"一点余量都没有"的下限。详见 `docs/nav2.md` 的"能过 ≠ 好过"一节。

三条可选修法（都还没做，做了要重跑整轮验收）：

1. **把缝加宽**（斜墙往西挪 0.2 m，或把它缩短）→ 世界恢复成"设计好的样子"，
   代价是 `maps/rooms_obstacles.*` 要重新建（约 6 分钟）；
2. **把 `inflation_radius` 降到 0.25 左右** → 窄通道能过，但贴墙余量变小，
   会影响**所有**导航行为，得重跑空场地那一轮回归；
3. 换规划器插件或加大 `backup` 恢复距离，让它卡住时能退出来 —— 治标。

---

## 怎么复现

### 第一轮：空房间 `rooms.sdf`

```bash
# 起一套带 RViz 的 Nav2（Gazebo + 地图 + AMCL）
ros2 launch my_robot_description nav.launch.py

# ① 起点到终点
ros2 run my_robot_description nav_goals.py -2.5,0.6 -2.5,1.9 2.5,1.9 3.0,0.3

# ② 障碍物避让（T1：静态未知障碍物挡路）
ros2 run my_robot_description obs_test.py --goal=0.0,0.0 --obstacle=-2.0,0.0

# ②（T5：细杆）
ros2 run my_robot_description obs_test.py --goal=0.0,0.0 --obstacle=-2.0,0.0 --size 0.05

# ②（T2：车前 0.7 m 突现 —— 这一条目前会卡死）
ros2 run my_robot_description obs_test.py --goal=-4.0,0.0 --obstacle=-2.0,0.0 --trigger-dist 0.7

# ③ 路径点导航
ros2 run my_robot_description nav_goals.py --mode follow_waypoints -2.5,0.6 -2.5,1.9 2.5,1.9 3.0,0.3
```

### 第二轮：带障碍物的新世界 `rooms_obstacles.sdf`

```bash
# 1) 建图（边建图边导航 + 自动覆盖路线，不用人遥控），完了存图
ros2 launch my_robot_description nav.launch.py slam:=true world:=rooms_obstacles.sdf rviz:=false
ros2 run my_robot_description nav_goals.py --file $(ros2 pkg prefix my_robot_description)/share/my_robot_description/routes/rooms_obstacles_coverage.txt \
    --mode follow_waypoints --timeout 1500
ros2 run nav2_map_server map_saver_cli -f src/my_robot_description/maps/rooms_obstacles

# 2) 查地图建得对不对（含 5 个障碍物有没有进图），顺便出对照图
ros2 run my_robot_description mapeval.py src/my_robot_description/maps/rooms_obstacles.yaml \
    --world src/my_robot_description/worlds/rooms_obstacles.sdf \
    --png src/my_robot_description/docs/rooms_obstacles_map.png

# 3) 换新地图重起 Nav2，跑三条验收
ros2 launch my_robot_description nav.launch.py sim:=true world:=rooms_obstacles.sdf \
    map:=$(ros2 pkg prefix my_robot_description)/share/my_robot_description/maps/rooms_obstacles.yaml

# ① + ②（出生点 -> 房间B东南角，门AB 被封死，必绕 2.9 倍）
ros2 run my_robot_description obs_test.py --static --goal=4.2,-1.9

# ②（细杆：出生点 -> (1.5,0.3)，直线 1.66 m，实际要绕 12 m）
ros2 run my_robot_description obs_test.py --static --goal=1.5,0.3

# ③-A 五个可达路点
ros2 run my_robot_description nav_goals.py --file $(ros2 pkg prefix my_robot_description)/share/my_robot_description/routes/rooms_obstacles_waypoints_a.txt \
    --mode follow_waypoints --timeout 900

# ③-B 中间夹一个落在障碍物里的路点
ros2 run my_robot_description nav_goals.py --file $(ros2 pkg prefix my_robot_description)/share/my_robot_description/routes/rooms_obstacles_waypoints_b.txt \
    --mode follow_waypoints --timeout 900

# 想不通"为什么规划器不走这条能过的路"时，看代价图
ros2 run my_robot_description cost_slice.py 4.0,4.95,-2.4,-0.85
```

`obs_test.py` 动态模式会在测试结束后自己删掉生成的障碍物；万一中途中断，
手动删：`gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "obs_box", type: 2'`。
