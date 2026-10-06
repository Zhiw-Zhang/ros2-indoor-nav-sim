# 功能验收：自主移动 / 障碍物避让 / 路径点导航

对应三条需求。**每条都给了测试方法和实测数据**，包括一条没通过的。

| # | 要求 | 结论 | 一句话 |
|---|---|---|---|
| ① | 起点到终点自主移动 | ✅ **达标** | 两个房间 + 三个门 + 走廊，6/6、4/4、4/4 三组全成功 |
| ② | 障碍物避让 | ⚠️ **部分达标** | 静态未知障碍物（含 5 cm 细杆）干净绕开；**近距离突现会卡死** |
| ③ | 路径点导航 | ✅ **达标** | 一次调用走完 4 个路点；某点不可达时按配置跳过继续 |

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

## ② 障碍物避让

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

## 怎么复现

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

`obs_test.py` 会在测试结束后自己删掉生成的障碍物；万一中途中断，
手动删：`gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "obs_box", type: 2'`。
