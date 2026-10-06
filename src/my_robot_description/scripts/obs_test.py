#!/usr/bin/env python3
"""障碍物避让验收。两种模式：

**A. 动态模式（默认）**——在**运行时**生成一个障碍物，看 Nav2 怎么反应。

    # T1 静态未知障碍物挡在路径上（一开始就在）
    ros2 run my_robot_description obs_test.py --goal=-4.0,0.0 --obstacle=-2.0,0.0

    # T2 车开到一半，障碍物在车前 1 m 突然出现
    ros2 run my_robot_description obs_test.py --goal=-4.0,0.0 --obstacle=-2.0,0.0 --trigger-dist 1.0

    # T5 细杆
    ros2 run my_robot_description obs_test.py --goal=-4.0,0.0 --obstacle=-2.0,0.0 --size 0.05

    # 不生成障碍物的基线，用于对照
    ros2 run my_robot_description obs_test.py --goal=-4.0,0.0

**B. 静态模式（--static）**——障碍物是 world 文件里**固有**的（名字以 `obs_` 开头），
不生成也不删除，只看机器人怎么绕过它们。

    ros2 run my_robot_description obs_test.py --static --goal=4.2,-1.9 \
        --world ~/sim_nav_ws/install/.../worlds/rooms_obstacles.sdf

    不给 --world 时默认用包内 worlds/rooms_obstacles.sdf。

为什么要写脚本而不是几条命令行：要判"有没有撞上"、"collision_monitor 有没有介入"，
必须同步记录机器人位姿、/cmd_vel 和 /collision_monitor_state 的时间序列，
再和障碍物位置算最小间距。手敲命令做不到。

动态模式用 `ros2 run ros_gz_sim create` 生成障碍物（Gazebo 的 /world/<world>/create
服务），所以**不进地图**——这正是"地图上没有、雷达突然看到"的场景。
静态模式反过来：障碍物在 world 里、也在地图上，验的是"地图上已知的障碍会不会被绕开"。

真值位姿
--------
静态模式额外订阅 `/world/default/dynamic_pose/info`（SceneBroadcaster 发的世界坐标），
用它而不是 TF 里的位姿做接触判定。原因：TF 的 odom→base_link 来自 DiffDrive
的轮速积分，原地转时因侧滑，转角是真值的 1.37 倍，拿它算"有没有撞到"会得出错结论。
两条位姿的差在开头会打印出来（同一个仿真，正常应该只差几厘米的定位误差），
差得太多说明这次数据不可信。

判读口径
--------
* `最小间距` 是机器人中心到障碍物**真实矩形**（含旋转）的距离。
  车体外接圆半径 27.3 cm、内切半径 18.5 cm，所以：
    间距 > 27.3 cm → **确定没碰上**；< 0（车心进到障碍物里）→ 确定撞了；
    中间值可能是擦碰，需要看具体几何。
* `接触判定` 用 OBB-OBB 分离轴测试（车体 0.40×0.37 按真实朝向，障碍物也按
  真实朝向），比上面那个圆半径判据准 —— 直接看它。
* `collision_monitor` 用 APPROACH 策略（配置里 action_type: "approach"），
  触发时 `action_type` 会变成 3。
* `代价图标记` 取障碍物中心 15 cm 内全局代价图的最大代价，
  100 是致命障碍（说明雷达看到并标进去了；这条话题被缩放到 0~100）。
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

# Nav2 配置里的 footprint 是 [[-0.20,-0.185],[0.20,-0.185],[0.20,0.185],[-0.20,0.185]]
CHASSIS_HALF_LEN, CHASSIS_HALF_WID = 0.200, 0.185

BOX_SDF = ('<sdf version="1.9"><model name="{name}"><static>true</static>'
           '<link name="link">'
           '<collision name="c"><geometry><box><size>{s} {s} {h}</size></box></geometry></collision>'
           '<visual name="v"><geometry><box><size>{s} {s} {h}</size></box></geometry>'
           '<material><ambient>0.9 0.2 0.2 1</ambient><diffuse>0.9 0.2 0.2 1</diffuse></material>'
           '</visual></link></model></sdf>')

CM_NAMES = {0: 'DO_NOTHING', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH', 4: 'LIMIT'}


def parse_xy(s):
    p = s.replace(';', ',').split(',')
    return float(p[0]), float(p[1])


def rect_clearance(px, py, x0, x1, y0, y1):
    """点到矩形的距离；点在矩形内返回负值（到最近边的距离取负）。"""
    dx = max(x0 - px, px - x1, 0.0)
    dy = max(y0 - py, py - y1, 0.0)
    if dx == 0.0 and dy == 0.0:
        return -min(px - x0, x1 - px, py - y0, y1 - py)
    return math.hypot(dx, dy)


def default_world():
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('my_robot_description'),
                         'worlds', 'rooms_obstacles.sdf')
        if os.path.isfile(p):
            return p
    except Exception:                                      # noqa: BLE001
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    p = os.path.join(here, '..', 'worlds', 'rooms_obstacles.sdf')
    return p if os.path.isfile(p) else None


def parse_boxes(sdf_path):
    """读 world 里每个 model 的第一个 box 碰撞体，返回 (name, cx, cy, sx, sy, yaw)。

    ★ 这里**必须保留 yaw**，不能像 mapeval.parse_world 那样转成 AABB。
    斜着放的障碍物（rooms_obstacles.sdf 里的 obs_roomB_slant 转 25°）用
    AABB 判接触会误报：那个 1.60x0.35 的盒子转 25° 后 AABB 是 1.60x0.99，
    面积 1.59 m²，而实体只有 0.56 m² —— AABB 的四个角伸出去一大截，
    机器人明明从旁边 20 cm 处过去也会被判成"撞上了"（踩过这个坑）。
    """
    import xml.etree.ElementTree as ET                  # noqa: PLC0415
    root = ET.parse(sdf_path).getroot()
    boxes = []
    for model in root.iter('model'):
        name = model.get('name', '?')
        pose = [float(v) for v in (model.findtext('pose') or '').split()]
        while len(pose) < 6:
            pose.append(0.0)
        for box in model.iter('box'):                   # <plane> 里没有 box，自然跳过
            size_el = box.find('size')
            if size_el is None:
                continue
            sx, sy = [float(v) for v in size_el.text.split()][:2]
            boxes.append((name, pose[0], pose[1], sx, sy, pose[5]))
            break
    if not boxes:
        raise SystemExit(f'{sdf_path}: 没找到任何 box 碰撞体')
    return boxes


def load_solids(world_sdf):
    """返回 (障碍物, 结构墙)，都是 (name, cx, cy, sx, sy, yaw)。

    obstacles 是名字以 obs_ 开头的那些（与 mapeval.py 同一套约定）。
    """
    boxes = parse_boxes(world_sdf)
    obs = [b for b in boxes if b[0].startswith('obs_')]
    walls = [b for b in boxes if not b[0].startswith('obs_')]
    if not obs:
        raise SystemExit(f'{world_sdf}: 没有任何 obs_* 障碍物')
    return obs, walls


def point_to_box(px, py, box):
    """点到矩形（可旋转）的距离；点在矩形内返回负值。"""
    _, cx, cy, sx, sy, yaw = box
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = px - cx, py - cy
    lx, ly = dx * c + dy * s, -dx * s + dy * c      # 转到矩形自身坐标系
    ox, oy = max(abs(lx) - sx / 2, 0.0), max(abs(ly) - sy / 2, 0.0)
    if ox > 0 or oy > 0:
        return math.hypot(ox, oy)
    return -min(sx / 2 - abs(lx), sy / 2 - abs(ly))


def _box_corners(box):
    _, cx, cy, sx, sy, yaw = box
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + ex * c - ey * s, cy + ex * s + ey * c)
            for ex in (-sx / 2, sx / 2) for ey in (-sy / 2, sy / 2)]


def _box_axes(box):
    c, s = math.cos(box[5]), math.sin(box[5])
    return [(c, s), (-s, c)]


def obb_hit(a, b):
    """SAT：两个可旋转矩形是否相交（车体也是一样处理的矩形）。"""
    ca, cb = _box_corners(a), _box_corners(b)
    for ax, ay in _box_axes(a) + _box_axes(b):
        pa = [p[0] * ax + p[1] * ay for p in ca]
        pb = [p[0] * ax + p[1] * ay for p in cb]
        if max(pa) < min(pb) or max(pb) < min(pa):
            return False
    return True


def chassis(cx, cy, yaw):
    return ('robot', cx, cy, 2 * CHASSIS_HALF_LEN, 2 * CHASSIS_HALF_WID, yaw)


class ObsTest(Node):
    def __init__(self, args):
        super().__init__('obs_test')
        self.a = args
        self.obs_xy = parse_xy(args.obstacle) if args.obstacle else None
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose = None            # (x, y, yaw) in map frame（来自 TF，含定位误差）
        self.gz_pose = None         # (x, y, yaw) 仿真真值，世界坐标
        self.gz_n = 0               # dynamic_pose/info 里有几条
        self.gz_offsets = []        # 每帧 TF位姿 vs 真值 的平面距离（自检）
        self.cmd = (0.0, 0.0)       # (v, w)
        self.cm_state = 0
        self.cm_poly = ''
        self.cost_at_obs = None
        self.cost_cells = None
        self.cm_seen = []
        self.samples = []
        self.spawned_at = None

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                 self._on_cm, 10)
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._on_costmap, qos)
        if args.static:
            self.create_subscription(TFMessage, '/world/default/dynamic_pose/info',
                                     self._on_gzpose, 10)

    # ---- 回调 ----
    def _on_gzpose(self, m):
        """从 /world/<world>/dynamic_pose/info 里取机器人的**世界坐标**真值。

        实测坑：ros_gz_bridge 把 gz.msgs.Pose_V 转成 tf2_msgs/TFMessage 时
        会把实体名丢掉（frame_id / child_frame_id 全是空字符串），所以按名字
        找不到 my_robot，只能按顺序取第一条 —— 那条正是模型本身的世界位姿。
        后面跟着的 base_link 和 4 个轮子是**相对模型**的局部坐标（轮子在
        ±0.14/±0.17，车动它们也不变），拿来当位姿会得出"车一直没动"。

        取错了不会静默出错：run_static 全程对比 TF 位姿和这里的真值，
        偏差会打进报告（tf_vs_truth_*）。取错的话偏差会等于车的总位移。
        """
        self.gz_n = len(m.transforms)
        for tr in m.transforms:
            if 'my_robot' in tr.child_frame_id:
                self._set_gzpose(tr)
                return
        if m.transforms:                       # 名字被丢掉了：第一条就是模型
            self._set_gzpose(m.transforms[0])

    def _set_gzpose(self, tr):
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.gz_pose = (t.x, t.y, yaw)
        if self.pose:
            self.gz_offsets.append((self.pose[0] - t.x, self.pose[1] - t.y))

    def _on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def _on_cm(self, m):
        self.cm_state = int(m.action_type)
        self.cm_poly = m.polygon_name
        if self.spawned_at is not None and self.cm_state != 0:
            if not self.cm_seen or self.cm_seen[-1][1] != self.cm_state:
                self.cm_seen.append((time.time() - self.spawned_at, self.cm_state))

    def _on_costmap(self, m):
        if self.obs_xy is None:
            return
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        r, h, w = m.info.resolution, m.info.height, m.info.width
        x, y = self.obs_xy
        best, lethal, high, n = None, 0, 0, 0
        for dy in [i * 0.05 - 0.3 for i in range(13)]:
            for dx in [i * 0.05 - 0.3 for i in range(13)]:
                c = int((x + dx - ox) / r)
                rr = int((oy + h * r - (y + dy)) / r)
                if 0 <= rr < h and 0 <= c < w:
                    v = m.data[rr * w + c]
                    if v < 0:
                        continue
                    n += 1
                    if best is None or v > best:
                        best = v
                    # ★ 这条话题上的代价是**缩放到 0~100** 发布的，不是 0~255。
                    #   证据：地图里已知的墙（肯定致命）在这里读到的就是 100。
                    if v >= 100:
                        lethal += 1
                    elif v >= 50:
                        high += 1
        if best is not None:
            self.cost_at_obs = best
            self.cost_cells = (lethal, high, n)

    # ---- 工具 ----
    def update_tf(self):
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_link',
                                                 rclpy.time.Time())
        except Exception:                                  # noqa: BLE001
            return
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (t.x, t.y, yaw)

    def spawn(self):
        name = self.a.name
        sdf = BOX_SDF.format(name=name, s=self.a.size, h=self.a.height)
        x, y = self.obs_xy
        cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'default',
               '-name', name, '-string', sdf,
               '-x', str(x), '-y', str(y), '-z', str(self.a.height / 2 + 0.01),
               '-allow_renaming', 'false']
        print(f'>>> 在 ({x:+.2f}, {y:+.2f}) 生成障碍物 '
              f'{self.a.size}x{self.a.size}x{self.a.height} m', flush=True)
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        ok = 'Entity creation successful' in (r.stdout + r.stderr)
        print(f'    生成{"成功" if ok else "失败: " + r.stderr[-200:]}', flush=True)
        self.spawned_at = time.time()

    def remove(self):
        req = f'name: "{self.a.name}", type: 2'
        subprocess.run(['gz', 'service', '-s', '/world/default/remove',
                        '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '2000', '--req', req],
                       capture_output=True, text=True, timeout=30)

    def spin_for(self, seconds, fut=None):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.update_tf()
            if self.pose:
                self.samples.append((time.time(), self.pose[0], self.pose[1],
                                     self.cmd[0], self.cmd[1], self.cm_state))
            if fut is not None and fut.done():
                return True
        return fut is None or fut.done()


def run(args):
    rclpy.init()
    node = ObsTest(args)
    if not node.client.wait_for_server(timeout_sec=30.0):
        raise SystemExit('等不到 /navigate_to_pose —— Nav2 起来了吗？')

    x, y = parse_xy(args.goal)
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.w = 1.0

    t0 = time.time()
    fut = node.client.send_goal_async(goal)
    node.spin_for(30.0, fut)
    if not fut.done() or fut.result() is None or not fut.result().accepted:
        raise SystemExit('目标没有被接受')
    res_fut = fut.result().get_result_async()
    print(f'目标 ({x:+.2f}, {y:+.2f}) 已接受，开始跟踪', flush=True)

    # ★ 必须等 TF 真的就绪再记起点：spin_for 会在 goal 一被接受就返回，
    #   那一瞬间 tf buffer 可能还是空的，start 会变成 None，
    #   结果"横向偏移"永远算出 0（踩过）。
    t_pose = time.time()
    while node.pose is None and time.time() - t_pose < 15.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        node.update_tf()
    start = node.pose
    trig_logged = False
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        node.update_tf()
        if node.pose:
            node.samples.append((time.time(), node.pose[0], node.pose[1],
                                 node.cmd[0], node.cmd[1], node.cm_state))
        if node.obs_xy and node.spawned_at is None and node.pose:
            ob_x, ob_y = node.obs_xy
            d = math.hypot(node.pose[0] - ob_x, node.pose[1] - ob_y)
            if d <= args.trigger_dist:
                if not trig_logged:
                    print(f'    机器人距障碍物 {d:.2f} m，触发生成', flush=True)
                    trig_logged = True
                node.spawn()
        if res_fut.done():
            break
    elapsed = time.time() - t0
    status = int(res_fut.result().status) if res_fut.done() else -1

    # ---- 统计 ----
    out = {'goal': [x, y], 'status': status, 'seconds': round(elapsed, 1),
           'obstacle': None}
    status_name = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(status, f'未结束({status})')
    out['status_name'] = status_name

    if node.obs_xy:
        ox, oy = node.obs_xy
        half = args.size / 2
        x0, x1, y0, y1 = ox - half, ox + half, oy - half, oy + half
        post = [s for s in node.samples
                if node.spawned_at and s[0] >= node.spawned_at]
        cl = [rect_clearance(s[1], s[2], x0, x1, y0, y1) for s in post]
        min_cl = min(cl) if cl else float('nan')
        # 触发之后的横向偏移（相对起点->目标直线）
        lat = []
        if start:
            vx, vy = x - start[0], y - start[1]
            L = math.hypot(vx, vy) or 1.0
            for s in post:
                d = ((s[1] - start[0]) * vy - (s[2] - start[1]) * vx) / L
                lat.append(d)
        out['obstacle'] = {
            'at': [ox, oy], 'size': args.size, 'height': args.height,
            'trigger_dist': args.trigger_dist,
            'spawned': node.spawned_at is not None,
            'min_clearance_cm': round(100 * min_cl, 1) if cl else None,
            'max_lateral_cm': round(100 * max((abs(v) for v in lat), default=0.0), 1),
            'cost_at_obstacle': node.cost_at_obs,
            'cost_cells_lethal_high_total': node.cost_cells,
            'collision_monitor_states': [(round(t, 2), CM_NAMES.get(s, s))
                                         for t, s in node.cm_seen],
        }
        # 停车检测：触发后是否出现过 |v|,|w| 都约为 0 且不在终点
        stopped = False
        for s in post:
            if abs(s[3]) < 0.02 and abs(s[4]) < 0.05:
                if math.hypot(s[1] - x, s[2] - y) > 0.5:
                    stopped = True
                    break
        out['obstacle']['stopped_midway'] = stopped

    # ---- 打印 ----
    print('\n' + '=' * 62)
    print(f"目标 ({x:+.2f}, {y:+.2f})   结果: {status_name}   用时 {elapsed:.1f}s")
    o = out['obstacle']
    if o:
        if not o['spawned']:
            print('  ⚠ 障碍物**没有**生成（车没走到触发距离？）——本次结果无效')
        print(f"  障碍物 {o['size']}x{o['size']}x{o['height']} m @ ({o['at'][0]:+.2f}, {o['at'][1]:+.2f})"
              f"  触发距离 {o['trigger_dist']} m")
        mc = o['min_clearance_cm']
        if mc is None:
            print('  最小间距: 无数据')
        else:
            verdict = ('✅ 确定没碰上（> 外接半径 27.3 cm）' if mc > 27.3
                       else ('❌ 车心进到障碍物里了' if mc < 0
                             else '⚠ 擦碰风险区（0~27.3 cm），需看几何'))
            print(f"  机器人中心到障碍物 最小间距: {mc} cm   {verdict}")
        cc = o['cost_cells_lethal_high_total']
        print(f"  全局代价图（障碍物周围 0.6x0.6 m 内，共 {cc[2] if cc else '?'} 格）:")
        print(f"     最高代价 {o['cost_at_obstacle']}（这条话题缩放到 0~100，"
              f"100 = 致命）；致命格 {cc[0] if cc else '?'} 个，"
              f"中高代价(50~99) {cc[1] if cc else '?'} 个")
        print("     有致命格就说明：雷达把**地图上不存在**的障碍物标进代价图了")
        print(f"  相对起终点直线的最大横向偏移: {o['max_lateral_cm']} cm")
        cm = o['collision_monitor_states']
        if cm:
            print(f"  collision_monitor 介入: " +
                  ', '.join(f'{n}@+{t}s' for t, n in cm))
        else:
            print('  collision_monitor **从未介入**')
        print(f"  中途停过车: {o['stopped_midway']}")

    node.remove()
    node.destroy_node()
    rclpy.shutdown()
    if args.json:
        print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0 if status == 4 else 1


def run_static(args):
    """障碍物是 world 里固有的：只观察，不生成也不删除。"""
    world = args.world or default_world()
    if not world or not os.path.isfile(world):
        raise SystemExit(f'找不到 world 文件: {world}')
    obs, walls = load_solids(world)
    print(f'静态模式  world = {world}')
    print(f'  障碍物 {len(obs)} 个: ' + ', '.join(o[0] for o in obs), flush=True)

    rclpy.init()
    node = ObsTest(args)
    if not node.client.wait_for_server(timeout_sec=30.0):
        raise SystemExit('等不到 /navigate_to_pose —— Nav2 起来了吗？')

    x, y = parse_xy(args.goal)
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.w = 1.0

    t0 = time.time()
    fut = node.client.send_goal_async(goal)
    node.spin_for(30.0, fut)
    if not fut.done() or fut.result() is None or not fut.result().accepted:
        raise SystemExit('目标没有被接受')
    res_fut = fut.result().get_result_async()
    print(f'目标 ({x:+.2f}, {y:+.2f}) 已接受，开始跟踪', flush=True)

    # 真值位姿要先到位，否则起点取不到（动态模式踩过这个坑）
    t_wait = time.time()
    while node.gz_pose is None and time.time() - t_wait < 20.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.gz_pose is None:
        raise SystemExit('收不到 /world/default/dynamic_pose/info —— '
                         'bridge 里加上它了吗？（gazebo_sim.launch.py 的 bridge 参数）')
    start = node.gz_pose

    samples = []
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        node.update_tf()
        if node.gz_pose:
            samples.append((time.time(),) + tuple(node.gz_pose) +
                           (node.cmd[0], node.cmd[1], node.cm_state))
        if res_fut.done():
            break
    elapsed = time.time() - t0
    status = int(res_fut.result().status) if res_fut.done() else -1
    status_name = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(
        status, f'未结束({status})')

    # ---- 统计 ----
    def scan(group):
        rows = []
        for box in group:
            cl = [point_to_box(s[1], s[2], box) for s in samples]
            touch = any(obb_hit(chassis(s[1], s[2], s[3]), box) for s in samples)
            rows.append({'name': box[0], 'min_clearance_cm': round(100 * min(cl), 1) if cl else None,
                         'touched': touch})
        return rows

    obs_rows = scan(obs)
    wall_rows = scan(walls)
    travel = sum(math.hypot(samples[i][1] - samples[i - 1][1],
                            samples[i][2] - samples[i - 1][2])
                 for i in range(1, len(samples)))
    straight = math.hypot(x - start[0], y - start[1])
    final_err = None
    if samples:
        final_err = math.hypot(samples[-1][1] - x, samples[-1][2] - y)
    lat = max((abs((s[1] - start[0]) * (y - start[1]) - (s[2] - start[1]) * (x - start[0]))
               / straight) for s in samples) if samples and straight > 1e-6 else 0.0

    # 自检：TF 位姿（含 SLAM/AMCL 的定位误差）和仿真真值差多少。
    # 差值要拆成两部分看：
    #   * 系统偏移 —— 地图坐标系原点由 SLAM 决定，和仿真世界原点不会严格重合，
    #     所以两组坐标之间本来就有一个近似恒定的平移。目标点是按地图坐标给的，
    #     这个常数偏移不影响导航成败。
    #   * 抖动（扣掉系统偏移后的残差）—— 这才是定位质量。取错
    #     dynamic_pose/info 里那条位姿的话，残差会等于车的总位移（十几米）。
    off = node.gz_offsets
    off_stats = None
    if off:
        mx = sum(d[0] for d in off) / len(off)
        my = sum(d[1] for d in off) / len(off)
        resid = [math.hypot(d[0] - mx, d[1] - my) for d in off]
        off_stats = {'dx_cm': round(100 * mx, 1), 'dy_cm': round(100 * my, 1),
                     'systematic_cm': round(100 * math.hypot(mx, my), 1),
                     'residual_mean_cm': round(100 * sum(resid) / len(resid), 1),
                     'residual_max_cm': round(100 * max(resid), 1), 'n': len(off)}

    out = {'mode': 'static', 'world': world, 'goal': [x, y],
           'status': status, 'status_name': status_name,
           'seconds': round(elapsed, 1),
           'start': [round(v, 3) for v in start],
           'final_error_cm': round(100 * final_err, 1) if final_err is not None else None,
           'path_length_m': round(travel, 2),
           'straight_line_m': round(straight, 2),
           'detour_ratio': round(travel / straight, 2) if straight > 1e-6 else None,
           'max_lateral_cm': round(100 * lat, 1),
           'gz_transforms_per_msg': node.gz_n,
           'tf_vs_truth_cm': off_stats,
           'obstacles': obs_rows, 'walls': wall_rows,
           'collision_monitor_states': [(round(t - t0, 2), CM_NAMES.get(s, s))
                                        for t, s in node.cm_seen],
           'samples': len(samples)}

    # ---- 打印 ----
    print('\n' + '=' * 66)
    print(f"目标 ({x:+.2f}, {y:+.2f})   结果: {status_name}   用时 {elapsed:.1f}s")
    if off_stats:
        flag = '✅' if off_stats['residual_max_cm'] <= 10 else '⚠'
        print(f"  {flag} 定位自检（TF 位姿 vs 仿真真值，{off_stats['n']} 帧）: "
              f"系统偏移 {off_stats['systematic_cm']} cm"
              f"（dx {off_stats['dx_cm']:+.1f}, dy {off_stats['dy_cm']:+.1f}）"
              f"，扣掉它之后抖动 最大 {off_stats['residual_max_cm']} cm"
              f" / 平均 {off_stats['residual_mean_cm']} cm")
        if off_stats['residual_max_cm'] > 50:
            print(f"     ⚠ 残差过大 —— 真值位姿可能取错了 transform"
                  f"（本消息共 {node.gz_n} 条），下面的接触判定不可信")
    print(f"  路径长度 {out['path_length_m']} m（直线 {out['straight_line_m']} m，"
          f"绕行 {out['detour_ratio']}x）  最大横向偏移 {out['max_lateral_cm']} cm")
    print(f"  终点误差: {out['final_error_cm']} cm" if final_err is not None else '  终点误差: ?')

    print(f"\n-- 障碍物最小间距（机器人中心到矩形；> 27.3 cm 确定没碰上）--")
    for r in obs_rows:
        mc = r['min_clearance_cm']
        v = ('✅ 确定没碰上' if mc > 27.3 else
             ('❌ 车心进到障碍物里' if mc < 0 else '⚠ 擦碰风险区，看下面的 SAT 判定'))
        print(f"  {'☠' if r['touched'] else ' '} {r['name']:<18s} 最小间距 {mc:6.1f} cm"
              f"   {v}")

    print(f"\n-- 接触判定（车体 0.40x0.37 按真实朝向做 SAT 分离轴测试）--")
    hit_obs = [r['name'] for r in obs_rows if r['touched']]
    hit_wall = [r['name'] for r in wall_rows if r['touched']]
    print(f"  障碍物: {'❌ 撞到了 ' + ', '.join(hit_obs) if hit_obs else '✅ 一次都没碰到'}")
    print(f"  墙体  : {'❌ 撞到了 ' + ', '.join(hit_wall) if hit_wall else '✅ 一次都没碰到'}")
    near = min((r['min_clearance_cm'] for r in wall_rows), default=None)
    if near is not None:
        print(f"  （到最近墙体的全局最小间距 {near:.1f} cm）")

    cm = out['collision_monitor_states']
    print(f"\n  collision_monitor: " + (', '.join(f'{n}@+{t}s' for t, n in cm)
                                        if cm else '**从未介入**'))

    node.destroy_node()
    rclpy.shutdown()
    if args.json:
        print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0 if (status == 4 and not hit_obs and not hit_wall) else 1


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--goal', required=True, help='x,y')
    ap.add_argument('--obstacle', help='动态模式：障碍物位置 x,y；不给就只跑基线')
    ap.add_argument('--static', action='store_true',
                    help='静态模式：障碍物来自 world 里的 obs_*（不生成/不删除）')
    ap.add_argument('--world', default=None,
                    help='静态模式用的 world SDF，默认包内 worlds/rooms_obstacles.sdf')
    ap.add_argument('--size', type=float, default=0.4, help='动态模式：障碍物边长（默认 0.4）')
    ap.add_argument('--height', type=float, default=0.6, help='动态模式：障碍物高度（默认 0.6）')
    ap.add_argument('--trigger-dist', type=float, default=999.0,
                    help='动态模式：机器人距障碍物多少米时生成（默认一开始就生成）')
    ap.add_argument('--name', default='obs_box', help='动态模式：Gazebo 实体名')
    ap.add_argument('--timeout', type=float, default=240.0, help='总超时秒数')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args(argv)
    return run_static(a) if a.static else run(a)


if __name__ == '__main__':
    sys.exit(main())
