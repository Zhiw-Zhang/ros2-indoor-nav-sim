#!/usr/bin/env python3
"""障碍物避让验收：在**运行时**生成一个障碍物，看 Nav2 怎么反应。

    # T1 静态未知障碍物挡在路径上（一开始就在）
    ros2 run my_robot_description obs_test.py --goal=-4.0,0.0 --obstacle=-2.0,0.0

    # T2 车开到一半，障碍物在车前 1 m 突然出现
    ros2 run my_robot_description obs_test.py --goal=-4.0,0.0 --obstacle=-2.0,0.0 --trigger-dist 1.0

    # T5 细杆
    ros2 run my_robot_description obs_test.py --goal=-4.0,0.0 --obstacle=-2.0,0.0 --size 0.05

    # 不生成障碍物的基线，用于对照
    ros2 run my_robot_description obs_test.py --goal=-4.0,0.0

为什么要写脚本而不是几条命令行：要判"有没有撞上"、"collision_monitor 有没有介入"，
必须同步记录机器人位姿、/cmd_vel 和 /collision_monitor_state 的时间序列，
再和障碍物位置算最小间距。手敲命令做不到。

障碍物用 `ros2 run ros_gz_sim create` 生成（Gazebo 的 /world/<world>/create 服务），
所以**不进地图**——这正是"地图上没有、雷达突然看到"的场景。

判读口径
--------
* `最小间距` 是机器人中心到障碍物矩形的距离。
  车体外接圆半径 27.3 cm、内切半径 18.5 cm，所以：
    间距 > 27.3 cm → **确定没碰上**；< 0（车心进到障碍物里）→ 确定撞了；
    中间值可能是擦碰，需要看具体几何。
* `collision_monitor` 用 APPROACH 策略（配置里 action_type: "approach"），
  触发时 `action_type` 会变成 3。
* `代价图标记` 取障碍物中心 15 cm 内全局代价图的最大代价，
  253 以上是致命障碍（说明雷达看到并标进去了）。
"""
import argparse
import json
import math
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
from tf2_ros import Buffer, TransformListener

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


class ObsTest(Node):
    def __init__(self, args):
        super().__init__('obs_test')
        self.a = args
        self.obs_xy = parse_xy(args.obstacle) if args.obstacle else None
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose = None            # (x, y, yaw) in map frame
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

    # ---- 回调 ----
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


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--goal', required=True, help='x,y')
    ap.add_argument('--obstacle', help='障碍物位置 x,y；不给就只跑基线')
    ap.add_argument('--size', type=float, default=0.4, help='障碍物边长（默认 0.4）')
    ap.add_argument('--height', type=float, default=0.6, help='障碍物高度（默认 0.6）')
    ap.add_argument('--trigger-dist', type=float, default=999.0,
                    help='机器人距障碍物多少米时生成（默认一开始就生成）')
    ap.add_argument('--name', default='obs_box', help='Gazebo 实体名')
    ap.add_argument('--timeout', type=float, default=240.0, help='总超时秒数')
    ap.add_argument('--json', action='store_true')
    return run(ap.parse_args(argv))


if __name__ == '__main__':
    sys.exit(main())
