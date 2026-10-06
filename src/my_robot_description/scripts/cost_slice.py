#!/usr/bin/env python3
"""把一张占据栅格（代价图）在指定矩形区域里打成 ASCII 表。

    # 看全局代价图在某个区域长什么样
    ros2 run my_robot_description cost_slice.py 4.0,4.95,-2.4,-0.85

    # 看别的图层
    ros2 run my_robot_description cost_slice.py -0.3,0.3,1.2,2.45 \
        --topic /global_costmap/static_layer

为什么需要它
------------
"这条路到底能不能过"不能只看世界的几何尺寸。规划器看到的是**代价图**：
障碍物周围按 `inflation_radius` 铺开一圈代价，越靠近越高。两个障碍物之间
哪怕物理上留了 0.7 m，如果两边各铺 0.35 m 的膨胀，中间就只剩一条极窄的
低代价缝 —— 看起来"能过"，实际规划器几乎无路可走。

实测例子（rooms_obstacles.sdf 的走廊，障碍物与北墙之间留 0.675 m）：
最便宜的格子代价是 **71**，两侧 0.25 m 外就是 99（内切膨胀），全程没有一格
是自由空间。代价读数的含义：

    0      自由
    1~98   膨胀代价（越大越靠近障碍物）
    99     内切膨胀（INSCRIBED_INFLATED_OBSTACLE，车心在这里车体就已经压到障碍物）
    100    致命（LETHAL_OBSTACLE）
    -1     未知

★ `nav_msgs/OccupancyGrid` 的 `data[0]` 是**左下角**（y 最小），和 pgm 图像
  "第 0 行在顶部"相反。按图像习惯算 row 会把整张表上下翻转，看起来就像
  "墙画错位置了"（踩过）。
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid


def parse_rect(text):
    v = [float(x) for x in text.replace(';', ',').split(',')]
    if len(v) != 4:
        raise SystemExit('矩形要写成 x0,x1,y0,y1')
    x0, x1, y0, y1 = v
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    return x0, x1, y0, y1


def _protect_negative_rect(argv):
    """把"看起来像 x0,x1,y0,y1"的位置参数挪到 `--` 之后。

    argparse 只在**整个 token 就是个数字**时才认负数，`-0.15,0.15,2.0,2.2`
    因为带逗号会被当成选项；而 `ros2 run` 又会把用户写的 `--` 吞掉
    （和 `nav_goals.py` 同一个坑）。不能"插个 `--` 就完事"：那样后面所有
    东西都变成位置参数，`--step 0.05` 会被当成矩形。
    """
    def is_rect(t):
        if not t.startswith('-'):
            return False
        parts = t.replace(';', ',').split(',')
        if len(parts) != 4:
            return False
        try:
            [float(p) for p in parts]
        except ValueError:
            return False
        return True

    rects = [a for a in argv if a != '--' and is_rect(a)]
    if not rects:
        return argv
    rest = [a for a in argv if a != '--' and not is_rect(a)]
    return rest + ['--'] + rects


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('rect', help='x0,x1,y0,y1（米，map 坐标系）')
    ap.add_argument('--topic', default='/global_costmap/costmap',
                    help='要看的栅格话题，默认 /global_costmap/costmap')
    ap.add_argument('--step', type=float, default=None, help='采样步长，默认取分辨率')
    ap.add_argument('--timeout', type=float, default=30.0, help='等话题的秒数')
    a = ap.parse_args(_protect_negative_rect(argv if argv is not None else sys.argv[1:]))
    x0, x1, y0, y1 = parse_rect(a.rect)

    rclpy.init()
    node = Node('cost_slice')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    got = {}
    node.create_subscription(OccupancyGrid, a.topic,
                             lambda m: got.setdefault('m', m), qos)
    t0 = time.time()
    while 'm' not in got and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    if 'm' not in got:
        raise SystemExit(f'{a.timeout:.0f}s 内收不到 {a.topic}'
                         f'（话题存在吗？transient_local 的 QoS 对上了吗？）')

    m = got['m']
    i = m.info
    ox, oy = i.origin.position.x, i.origin.position.y
    res = i.resolution
    step = a.step or res
    d = np.array(m.data, np.int16).reshape(i.height, i.width)
    print(f'{a.topic}   {i.width}x{i.height} @ {res} m   origin ({ox:.3f}, {oy:.3f})')
    print(f'区域 x[{x0},{x1}] y[{y0},{y1}]  步长 {step} m')
    print('         0=自由 1~98=膨胀代价 99=内切膨胀 100=致命 -1=未知')
    print('       ' + ''.join(f'{x:6.2f}' for x in np.arange(x0, x1 + 1e-9, step)))
    for yy in np.arange(y1, y0 - 1e-9, -step):
        row = int((yy - oy) / res)                 # ← 左下角为原点，不是图像顶行
        cells = []
        for xx in np.arange(x0, x1 + 1e-9, step):
            c = int((xx - ox) / res)
            cells.append(f'{d[row, c]:6d}' if 0 <= row < i.height and 0 <= c < i.width
                         else '   out')
        print(f'{yy:6.2f}' + ''.join(cells))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
