#!/usr/bin/env python3
"""依次下发一串 NavigateToPose 目标，记录每个的结果和终点误差。

    ros2 run my_robot_description nav_goals.py -2.5,0.6 -2.5,1.9 2.5,1.9 3.0,0.3
    ros2 run my_robot_description nav_goals.py --file goals.txt
    ros2 run my_robot_description nav_goals.py -2.5,1.9 --timeout 90 --json

每个目标写成 `x,y` 或 `x,y,yaw_deg`（度，0 = +x 方向）。`--file` 每行一个，
`#` 开头和空行会被忽略。

★ 负坐标（比如 `-2.5,0.6`）不用加 `--`：argparse 本来会把 `-2.5` 当成选项，
  而 `ros2 run` 又会把用户写的 `--` 吞掉，所以脚本内部会自己补一个。
  这是实际用起来才发现的坑——这张地图里负坐标是常态。

每个目标结束后会用 TF 查 `map` -> `base_link`（默认取最近可用的变换，
所以不依赖 use_sim_time），打印终点位姿和到目标的距离。

全部成功退出码 0，有任何一个失败退出码 1，便于脚本化。
"""
import argparse
import json
import math
import re
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import FollowWaypoints, NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


def parse_goal(text):
    parts = [p for p in text.replace(';', ',').split(',') if p.strip() != '']
    if len(parts) < 2:
        raise ValueError(f'目标格式应为 x,y[,yaw_deg]，收到 {text!r}')
    x, y = float(parts[0]), float(parts[1])
    yaw = math.radians(float(parts[2])) if len(parts) > 2 else 0.0
    return x, y, yaw


def load_goals(args):
    out = []
    for g in args.goals or []:
        out.append(parse_goal(g))
    if args.file:
        with open(args.file) as f:
            for line in f:
                line = line.split('#')[0].strip()
                if line:
                    out.append(parse_goal(line))
    if not out:
        raise SystemExit('没有目标。用法见 --help')
    return out


class GoalRunner(Node):
    def __init__(self, frame):
        super().__init__('nav_goals')
        self.frame = frame
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.wp_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.wp_now = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def spin_for(self, seconds, fut=None):
        """转 seconds 秒（或直到 fut 完成），期间保持 TF 回调运行。"""
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)
            if fut is not None and fut.done():
                return True
        return fut is None or fut.done()

    def lookup(self, retries=10):
        # ★ 刚连上时 buffer 可能还没填好，lookup 会报 "map ... does not exist"。
        #   重试几次，别把一次瞬时失败当成"查不到位姿"（踩过）。
        err = None
        for _ in range(retries):
            try:
                tr = self.tf_buffer.lookup_transform(
                    self.frame, 'base_link', rclpy.time.Time())
                break
            except Exception as e:                  # noqa: BLE001
                err = str(e)
                rclpy.spin_once(self, timeout_sec=0.1)
        else:
            return None, err
        t = tr.transform.translation
        q = tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw), None

    def run(self, goals, timeout):
        if not self.client.wait_for_server(timeout_sec=30.0):
            raise SystemExit('等不到 /navigate_to_pose action server —— Nav2 起来了吗？')
        results = []
        for i, (x, y, yaw) in enumerate(goals, 1):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = self.frame
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw / 2)
            goal.pose.pose.orientation.w = math.cos(yaw / 2)

            print(f'[{i}/{len(goals)}] 目标 ({x:+.3f}, {y:+.3f}) '
                  f'yaw {math.degrees(yaw):+.1f}° ...', flush=True)
            t0 = time.time()
            send_fut = self.client.send_goal_async(goal)
            self.spin_for(timeout, send_fut)
            if not send_fut.done() or send_fut.result() is None:
                print('    ✗ 发送超时'); results.append({'ok': False, 'why': 'send timeout'})
                continue
            handle = send_fut.result()
            if not handle.accepted:
                print('    ✗ 目标被拒绝'); results.append({'ok': False, 'why': 'rejected'})
                continue
            res_fut = handle.get_result_async()
            self.spin_for(timeout, res_fut)
            elapsed = time.time() - t0
            if not res_fut.done():
                print(f'    ✗ {timeout:.0f}s 内没返回结果')
                results.append({'ok': False, 'why': 'result timeout',
                                'seconds': elapsed})
                continue
            status = res_fut.result().status
            ok = (status == 4)          # GoalStatus.STATUS_SUCCEEDED
            pose, err = self.lookup()
            rec = {'goal': [x, y, math.degrees(yaw)], 'status': int(status),
                   'ok': ok, 'seconds': round(elapsed, 1)}
            if pose:
                dist = math.hypot(pose[0] - x, pose[1] - y)
                rec['final'] = [round(v, 3) for v in pose]
                rec['dist_m'] = round(dist, 3)
                print(f'    {"✅ SUCCEEDED" if ok else f"✗ status={status}"}  '
                      f'{elapsed:.1f}s  终点 ({pose[0]:+.3f}, {pose[1]:+.3f}, '
                      f'{math.degrees(pose[2]):+.1f}°)  到目标 {dist*100:.1f} cm')
            else:
                print(f'    {"✅ SUCCEEDED" if ok else f"✗ status={status}"}  '
                      f'{elapsed:.1f}s  (查不到 TF: {err})')
            results.append(rec)
        return results


    # ---- 路径点导航（nav2_waypoint_follower 的 FollowWaypoints）----
    def _wp_feedback(self, msg):
        cur = msg.feedback.current_waypoint
        if cur != self.wp_now:
            self.wp_now = cur
            print(f'    → 正在去第 {cur + 1} 个路点', flush=True)

    def run_waypoints(self, goals, timeout):
        """一次 FollowWaypoints 走完全部路点（区别于逐个发 NavigateToPose）。

        用的是 nav2_waypoint_follower，它的 stop_on_failure 参数决定
        "某个路点到不了"时是跳过继续还是整体失败（本工程配的是 false = 继续）。
        """
        if not self.wp_client.wait_for_server(timeout_sec=30.0):
            raise SystemExit('等不到 /follow_waypoints —— waypoint_follower 起来了吗？')
        goal = FollowWaypoints.Goal()
        for x, y, yaw in goals:
            ps = PoseStamped()
            ps.header.frame_id = self.frame
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            goal.poses.append(ps)

        print(f'一次下发 {len(goal.poses)} 个路点给 /follow_waypoints ...', flush=True)
        t0 = time.time()
        fut = self.wp_client.send_goal_async(goal, feedback_callback=self._wp_feedback)
        self.spin_for(timeout, fut)
        if not fut.done() or fut.result() is None or not fut.result().accepted:
            raise SystemExit('FollowWaypoints 目标没有接受')
        res_fut = fut.result().get_result_async()
        self.spin_for(timeout, res_fut)
        elapsed = time.time() - t0
        if not res_fut.done():
            raise SystemExit(f'{timeout:.0f}s 内 FollowWaypoints 没返回')
        missed_raw = list(res_fut.result().result.missed_waypoints)
        # ★ missed_waypoints 里是 MissedWaypoint 对象，不是序号。
        #   直接 `i in missed` 永远为假，会把失败的点也打成"到达"（踩过）。
        missed = [int(m.index) for m in missed_raw]
        print(f'\n结果: status={int(res_fut.result().status)}  '
              f'用时 {elapsed:.1f}s')
        for i, (x, y, _) in enumerate(goals):
            if i in missed:
                m = next(m for m in missed_raw if int(m.index) == i)
                code = int(m.error_code)
                print(f'  ❌ 未到达  #{i + 1}  ({x:+.3f}, {y:+.3f})   '
                      f'error_code={code}{WP_ERR.get(code, "")}')
            else:
                print(f'  ✅ 到达    #{i + 1}  ({x:+.3f}, {y:+.3f})')
        pose, err = self.lookup()
        if pose:
            print(f'  终点 ({pose[0]:+.3f}, {pose[1]:+.3f})')
        return missed, elapsed


# nav2 规划类错误码（来自 nav2_msgs/ComputePathToPose.action）。
# FollowWaypoints 的 MissedWaypoint.error_code 就是这一套。
WP_ERR = {
    200: ' UNKNOWN', 201: ' INVALID_PLANNER', 202: ' TF_ERROR',
    203: ' START_OUTSIDE_MAP', 204: ' GOAL_OUTSIDE_MAP', 205: ' START_OCCUPIED',
    206: ' GOAL_OCCUPIED', 207: ' TIMEOUT', 208: ' NO_VALID_PATH',
    600: ' UNKNOWN', 601: ' TASK_EXECUTOR_FAILED',
}


def _protect_negative_goals(argv):
    """在第一个"看起来像坐标"的参数（可能是负号开头）前插一个 `--`。

    argparse 会把 `-2.5,0.6` 当成选项；用户自己写 `--` 又会被 `ros2 run` 吞掉。
    所以在脚本内部补一个，用户直接写负坐标就行。
    """
    pat = re.compile(r'^-?\d+(\.\d+)?,-?\d+')
    out, inserted = [], False
    for a in argv:
        if not inserted and a != '--' and pat.match(a):
            out.append('--')
            inserted = True
        out.append(a)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('goals', nargs='*', help='x,y 或 x,y,yaw_deg')
    ap.add_argument('--file', help='目标列表文件，每行一个')
    ap.add_argument('--frame', default='map', help='目标坐标系，默认 map')
    ap.add_argument('--timeout', type=float, default=180.0, help='单个目标的超时秒数')
    ap.add_argument('--json', action='store_true', help='最后额外输出 JSON')
    ap.add_argument('--mode', choices=('sequence', 'follow_waypoints'),
                    default='sequence',
                    help='sequence=逐个发 NavigateToPose（默认）；'
                         'follow_waypoints=一次 FollowWaypoints 交给 nav2_waypoint_follower')
    a = ap.parse_args(_protect_negative_goals(
        sys.argv[1:] if argv is None else argv))

    goals = load_goals(a)
    rclpy.init()
    node = GoalRunner(a.frame)
    try:
        if a.mode == 'follow_waypoints':
            missed, elapsed = node.run_waypoints(goals, a.timeout)
            n_ok = len(goals) - len(missed)
            print(f'\n合计 {n_ok}/{len(goals)} 个路点到达')
            if a.json:
                print(json.dumps({'mode': a.mode, 'missed': missed,
                                  'seconds': round(elapsed, 1)},
                                 ensure_ascii=False, indent=2))
            return 0 if not missed else 1
        results = node.run(goals, a.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    n_ok = sum(1 for r in results if r.get('ok'))
    print(f'\n合计 {n_ok}/{len(results)} 成功')
    if a.json:
        print(json.dumps(results, ensure_ascii=False, indent=2))
    return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
