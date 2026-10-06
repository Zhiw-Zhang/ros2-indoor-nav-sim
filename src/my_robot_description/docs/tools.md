# 工具脚本

三个脚本装在 `lib/my_robot_description/` 下，都可以用 `ros2 run` 调用。
它们原来是调试时写在临时目录里的，后来收进仓库——**这类工具不该放在
`/tmp`**（被系统清过一次，丢了一批）。

```bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
```

---

## `mapeval.py` —— 建图质量验收

把一张栅格地图和 `worlds/*.sdf` 里的**真实几何**逐格比对，回答"这张图建得
对不对"。每次重新建图（`slam.launch.py` 或 `nav.launch.py slam:=true`）之后
都该跑一次。

```bash
ros2 run my_robot_description mapeval.py src/my_robot_description/maps/rooms.yaml
ros2 run my_robot_description mapeval.py ~/my_map.yaml --world src/my_robot_description/worlds/rooms.sdf
ros2 run my_robot_description mapeval.py ~/my_map.yaml --json      # 机器可读
ros2 run my_robot_description mapeval.py ~/my_map.yaml --no-shift  # 跳过平移搜索，快一些
```

**墙面和门洞是从 SDF 自动解析的**（只认 `<collision>` 里的 `<box>`，
`ground_plane` 那种 `<plane>` 自动忽略），门洞由"同一堵墙上共线两段 box
之间的空隙"推出。所以换 world 不用改这个脚本。

输出：

| 项目 | 含义 | 合格线 |
|---|---|---|
| 占用格到最近真实墙面 均值/最大 | 地图上的墙有没有画在真实的墙位置上 | 均值 < 2 cm，最大 < 10 cm |
| ≤ 5 cm / ≤ 10 cm 通过率 | 同上，分布视角 | ≤10 cm 必须 100% |
| 幻影墙 | 整块离真实墙 >15 cm 的连通分量 = 凭空多出来的东西 | 0 块 |
| 可见墙面覆盖率 | 真实墙面上有多少比例被画出来 | 越高越好，本场地 78~88% |
| 门洞通畅度 | 门洞里有多少占用格，理想 0 | < 25%；同方法量真实墙段是 40~70% |
| 最长墙中段 1 m 覆盖率 | 挑一段确实是墙的地方做对照 | —— |
| 最佳整体平移 | 地图相对 world 的整体偏移 | 应 < 5 cm |

退出码：任何门洞占用 ≥ 50% 就返回 1，方便脚本化卡关。

**本场地实测参考**（三张图，同一套判据）：

| 地图 | 到墙均值 | ≤ 5 cm | 幻影墙 | 覆盖率 |
|---|---|---|---|---|
| `slam:=true` 边导航边建 | **1.03 cm** | 99.42% | 0 | **86.0%** |
| 手动遥控建 | 1.12 cm | 99.95% | 0 | 87.1% |
| 早期脚本自动建（`maps/rooms.yaml`） | 1.50 cm | 85.70% | 0 | 78.2% |

---

## `nav_goals.py` —— 批量发导航目标并验收

依次下发一串 `NavigateToPose`，每个都打印耗时、终点位姿（TF 查
`map`->`base_link`）和到目标的距离。

```bash
ros2 run my_robot_description nav_goals.py -2.5,0.6 -2.5,1.9 2.5,1.9
ros2 run my_robot_description nav_goals.py --file goals.txt
ros2 run my_robot_description nav_goals.py 3.0,0.3,90 --timeout 90 --json

# 路径点导航：一次 FollowWaypoints 交给 nav2_waypoint_follower
ros2 run my_robot_description nav_goals.py --mode follow_waypoints -2.5,0.6 -2.5,1.9 2.5,1.9
```

★ 负坐标（`-2.5,0.6`）不用加 `--`：argparse 本来会把 `-2.5` 当选项，
而 `ros2 run` 又会把用户写的 `--` 吞掉，所以脚本内部会自己补一个。

`--mode follow_waypoints` 用的是 `nav2_waypoint_follower`，
`stop_on_failure` 决定某个路点到不了时是跳过继续还是整体失败
（本工程配的是 `false` = 跳过继续）。返回时会列出每个未到达路点及其
`error_code`（`204 GOAL_OUTSIDE_MAP` 之类），见 `docs/acceptance.md`。

* 目标格式 `x,y` 或 `x,y,yaw_deg`（度，0 = +x 方向）
* `--file` 每行一个，`#` 开头和空行忽略
* 查 TF 用的是"最近可用的变换"，所以**不需要** `use_sim_time`
* 全部成功退出码 0，有失败返回 1

`goals.txt` 例子（本场地串起三个门的一条路线）：

```
# x, y[, yaw_deg]
-2.5, 0.6
-2.5, 1.9      # 穿门 A 进走廊
 2.5, 1.9      # 走廊东端
 3.0, 0.3      # 穿门 B 进房间 B
```

---

## `obs_test.py` —— 障碍物避让验收

在**运行时**用 Gazebo 的 `/world/default/create` 服务生成一个障碍物，
同步记录机器人位姿、`/cmd_vel`、`/collision_monitor_state`，
最后给出"有没有撞上 / 兜底有没有介入 / 有没有绕开"的结论。

```bash
# T1 静态未知障碍物挡在路径正中（一开始就在）
ros2 run my_robot_description obs_test.py --goal=0.0,0.0 --obstacle=-2.0,0.0

# T2 车开到一半，障碍物在车前 0.7 m 突然出现
ros2 run my_robot_description obs_test.py --goal=-4.0,0.0 --obstacle=-2.0,0.0 --trigger-dist 0.7

# T5 细杆（验证稀疏点云会不会漏判）
ros2 run my_robot_description obs_test.py --goal=0.0,0.0 --obstacle=-2.0,0.0 --size 0.05

# 不生成障碍物的基线，用于对照
ros2 run my_robot_description obs_test.py --goal=-4.0,0.0
```

★ 参数值以 `-` 开头时必须写成 `--goal=-4.0,0.0` 这种 `=` 形式，
否则 argparse 会把它当成选项。

判读：

| 输出 | 含义 |
|---|---|
| 最小间距 > 27.3 cm | 车体外接圆半径是 27.3 cm，所以**确定没碰上** |
| 最小间距 < 0 | 车心进到障碍物矩形里了 = 撞了 |
| `collision_monitor 介入` | 兜底动作生效（本工程配的是 APPROACH，不是 STOP） |
| 全局代价图致命格 > 0 | 雷达把**地图上不存在**的障碍物标进代价图了 |

★ 代价图的坑：`/global_costmap/costmap` 上的代价是**缩放到 0~100** 发布的，
不是 0~255。判据是实测的——地图里已知的墙在这条话题上读到的就是 100。
所以**100 就是致命障碍**，不要拿 253 去比。

实测结论（含一条没通过的）见 `docs/acceptance.md`。

---

## `xwd2png.py` —— X11 截图转 png

纯调试辅助，跟机器人无关。WSLg 下想把手里的 RViz/Gazebo 窗口存成图片时用：

```bash
export DISPLAY=:0
xwininfo -root -tree | grep -i rviz        # 找到窗口 id，比如 0x600106
xwd -id 0x600106 -out /tmp/shot.xwd
ros2 run my_robot_description xwd2png.py /tmp/shot.xwd shot.png
```

坑：X 服务器的行距经常按 4 字节对齐（`bpl == w*4`），**即使头里写的是
`bpp=24`**。所以脚本按行距判断真实的每像素字节数，而不是信 `bpp`——照
`bpp=24` 解会 `cannot reshape array`。
