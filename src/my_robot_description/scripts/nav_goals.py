#!/usr/bin/env python3
"""依次下发一串 NavigateToPose 目标，记录每个的结果和终点误差。

    ros2 run my_robot_description nav_goals.py -2.5,0.6 -2.5,1.9 2.5,1.9 3.0,0.3
    ros2 run my_robot_description nav_goals.py --file goals.txt
    ros2 run my_robot_description nav_goals.py -2.5,1.9 --timeout 90 --json

每个目标写成 `x,y` 或 `x,y,yaw_deg`（度，0 = +x 方向）。`--file` 每行一个，
`#` 开头和空行会被忽略。

每个目标结束后会用 TF 查 `map` -> `base_link`（默认取最近可用的变换，
所以不依赖 use_sim_time），打印终点位姿和到目标的距离。

全部成功退出码 0，有任何一个失败退出码 1，便于脚本化。
"""
import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
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

    def lookup(self):
        try:
            tr = self.tf_buffer.lookup_transform(
                self.frame, 'base_link', rclpy.time.Time())
        except Exception as e:                      # noqa: BLE001
            return None, str(e)
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


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('goals', nargs='*', help='x,y 或 x,y,yaw_deg')
    ap.add_argument('--file', help='目标列表文件，每行一个')
    ap.add_argument('--frame', default='map', help='目标坐标系，默认 map')
    ap.add_argument('--timeout', type=float, default=180.0, help='单个目标的超时秒数')
    ap.add_argument('--json', action='store_true', help='最后额外输出 JSON')
    a = ap.parse_args(argv)

    goals = load_goals(a)
    rclpy.init()
    node = GoalRunner(a.frame)
    try:
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
