#!/usr/bin/env python3
"""
CPU 光线投射 2D 激光雷达 —— 替代 Gazebo 的 GPU 雷达传感器。

背景
----
在 WSL2 上，Gazebo Sim 8 的 Sensors 系统无法初始化渲染线程
（EGL surfaceless 平台不可用），会永久卡死并阻塞整个仿真服务器。
而 Gazebo 中所有雷达（lidar / gpu_lidar）都依赖渲染管线，没有 CPU 退路。

本节点用纯 CPU 光线投射生成等效的 sensor_msgs/LaserScan，
配合只跑物理引擎的 Gazebo（或不跑 Gazebo 也一样能用）。

位姿来源（很重要）
------------------
雷达需要知道自身位姿才能投射光束。可选:

  ground_truth  直接读取 Gazebo 原生 /world/default/pose/info 真实位姿（推荐）
  odom          使用 /odom，即差速驱动的轮式里程计
  integrated    完全自给自足：积分 /cmd_vel

【为什么默认不用 /odom】实测差速驱动的 /odom 是按轮子转速积分得到的：
机器人撞墙被挡住后轮子仍在转，/odom 会持续增长（实测 x 从 2.69 漂到 6.98），
而真实位置停在 2.69。用 /odom 会让雷达从错误的位置投射光束。

【为什么不走 ros_gz_bridge】实测 gz.msgs.Pose_V -> geometry_msgs/PoseArray
的桥接不会真正转发数据（ROS 侧 Publisher count 为 0），
因此本节点直接订阅 Gazebo 原生话题并解析其文本输出。
"""
import math
import re
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from my_robot_lidar_sim.geometry import raycast, rpy_to_quat, quat_to_yaw

try:
    import yaml
except ImportError:
    print("需要 python3-yaml: sudo apt install python3-yaml", file=sys.stderr)
    raise


class CpuLidarNode(Node):
    def __init__(self):
        super().__init__('cpu_lidar_node')

        # ---------------- 配置 ----------------
        self.declare_parameter('config_file', '')
        cfg_path = self.get_parameter('config_file').get_parameter_value().string_value
        if not cfg_path:
            raise RuntimeError('必须提供 config_file 参数')
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)

        lc = cfg['lidar']
        self.frame_id = lc['frame_id']
        self.topic = lc['topic']
        self.angle_min = float(lc['angle_min'])
        self.angle_max = float(lc['angle_max'])
        self.beam_count = int(lc['beam_count'])
        self.range_min = float(lc['range_min'])
        self.range_max = float(lc['range_max'])
        self.noise_stddev = float(lc.get('noise_stddev', 0.0))
        self.update_rate = float(lc['update_rate'])

        self.rng = np.random.default_rng(lc.get('random_seed', None))

        rc = cfg['robot']
        self.base_frame = rc['base_frame']
        self.odom_frame = rc['odom_frame']
        self.pose_source = str(rc.get('pose_source', 'ground_truth')).lower()
        self.gt_gz_topic = rc.get('ground_truth_gz_topic',
                                  '/world/default/pose/info')
        self.gt_index = int(rc.get('base_link_index', -1))
        self.gt_model_prefix = rc.get('base_link_name', 'my_robot/base_link')

        self.mount_xyz = [float(v) for v in lc.get('mount_xyz', [0.0, 0.0, 0.0])]
        self.mount_rpy = [float(v) for v in lc.get('mount_rpy', [0.0, 0.0, 0.0])]

        # 光束角度与角度增量
        self.angle_increment = (self.angle_max - self.angle_min) / \
            max(self.beam_count - 1, 1)
        self.beam_angles = self.angle_min + self.angle_increment * \
            np.arange(self.beam_count)

        # ---------------- 障碍物几何 ----------------
        boxes = cfg['world']['geometry']
        self.box_min = np.array(
            [[b['center'][0] - b['size'][0] / 2.0,
              b['center'][1] - b['size'][1] / 2.0] for b in boxes],
            dtype=np.float64).reshape(-1, 2)
        self.box_max = np.array(
            [[b['center'][0] + b['size'][0] / 2.0,
              b['center'][1] + b['size'][1] / 2.0] for b in boxes],
            dtype=np.float64).reshape(-1, 2)
        self.get_logger().info(f'载入 {self.box_min.shape[0]} 个障碍物用于光线投射')

        # ---------------- 状态 ----------------
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.odom_pose = None
        self.gt_pose = None
        self._gt_first_sample = None
        self._gt_auto_index = None
        self._gt_lock = threading.Lock()
        self._gt_proc = None
        self._gt_stop = threading.Event()
        self.last_cmd_time = self.get_clock().now()

        # ---------------- ROS 接口 ----------------
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.scan_pub = self.create_publisher(LaserScan, self.topic, scan_qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf = tf2_ros.StaticTransformBroadcaster(self)

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel,
                                 QoSProfile(depth=10))
        self.create_subscription(Odometry, 'odom', self.on_odom,
                                 QoSProfile(depth=10))

        self.publish_static_laser_tf()

        # 若使用真实位姿，启动 Gazebo 原生话题读取线程
        if self.pose_source in ('ground_truth', 'auto'):
            self._gt_thread = threading.Thread(
                target=self._ground_truth_loop, daemon=True)
            self._gt_thread.start()

        self.create_timer(1.0 / self.update_rate, self.on_scan_timer)
        self.create_timer(0.02, self.on_tf_timer)
        self.create_timer(5.0, self.check_pose_source)

        self.get_logger().info(
            f'CPU 激光雷达已启动: {self.topic} @ {self.update_rate} Hz, '
            f'{self.beam_count} 束, 量程 [{self.range_min}, {self.range_max}] m, '
            f'噪声 stddev={self.noise_stddev}, 位姿来源={self.pose_source}'
        )

    # ==================================================================
    # 位姿来源: 直接读取 Gazebo 原生 /world/default/pose/info
    # ==================================================================
    def _ground_truth_loop(self):
        """在后台线程里运行 `gz topic -e -t <topic>` 并解析实体位姿。

        直接读 Gazebo 原生话题，绕开 ros_gz_bridge 中不工作的
        Pose_V -> PoseArray 转换。
        """
        cmd = ['gz', 'topic', '-e', '-t', self.gt_gz_topic]
        try:
            self._gt_proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                text=True, bufsize=1)
        except FileNotFoundError:
            self.get_logger().error('找不到 gz 命令，无法读取真实位姿')
            return

        self.get_logger().info(
            f'开始读取 Gazebo 真实位姿: {" ".join(cmd)}')

        buf = []
        last_parse = 0.0
        prev_line = None
        for line in self._gt_proc.stdout:
            if self._gt_stop.is_set():
                break
            bare = line.rstrip('\n')

            # 消息起点: 顶格的 "header {"
            if not buf:
                if bare.strip() != 'header {':
                    continue
                buf.append('\n')  # 前置换行，配合解析用的 r'\npose\s*\{'

            buf.append(line)

            # 消息终点: 顶格的 "}"，且下一行不是 "pose {"。
            # 不能用"花括号计数归零"来判断：header 块自己就归零，
            # 会把整条 Pose_V 在 header 处截断，导致解析不到任何实体。
            if bare == '}':
                # 需要看下一行才能确定消息是否结束，因此延迟到下一轮
                prev_line = bare
                continue
            if prev_line == '}' and not bare.strip().startswith('pose {'):
                now = time.time()
                # 限流: 最多 20 Hz 解析，省 CPU
                if now - last_parse >= 0.05:
                    last_parse = now
                    try:
                        self._parse_pose_v(''.join(buf))
                    except Exception as exc:  # 解析失败不应杀死线程
                        self.get_logger().warn(
                            f'解析真实位姿失败: {exc}', throttle_duration_sec=10.0)
                buf = []
            prev_line = bare

    def _parse_pose_v(self, text):
        """解析 Gazebo Pose_V 文本，提取各实体的 name 与 position/orientation。

        注意: 顶层闭合的 "}" 顶格（第 0 列），而内层块如
        "  position { ... }" 是缩进的。因此这里必须用 "^} 顶格"
        作为块结束标志；若只用 "\\n}" 会把 position 块也当结束符，
        导致解析出的位置恒为 0。
        """
        poses = []
        # 切分顶层 pose { ... } 块：结束的 } 必须顶格
        for m in re.finditer(r'\npose\s*\{(.*?)\n\}', text, re.S):
            block = m.group(1)
            nm = re.search(r'name:\s*"([^"]*)"', block)
            if not nm:
                continue
            pos = re.search(r'position\s*\{([^}]*)\}', block, re.S)
            ori = re.search(r'orientation\s*\{([^}]*)\}', block, re.S)

            def num(blob, key):
                if not blob:
                    return 0.0
                mm = re.search(rf'\b{key}:\s*([-0-9.eE+]+)', blob)
                return float(mm.group(1)) if mm else 0.0

            poses.append({
                'name': nm.group(1),
                'x': num(pos.group(1) if pos else None, 'x'),
                'y': num(pos.group(1) if pos else None, 'y'),
                'z': num(pos.group(1) if pos else None, 'z'),
                'qx': num(ori.group(1) if ori else None, 'x'),
                'qy': num(ori.group(1) if ori else None, 'y'),
                'qz': num(ori.group(1) if ori else None, 'z'),
                'qw': num(ori.group(1) if ori else None, 'w'),
            })

        if not poses:
            return

        idx = self._resolve_ground_truth_index(poses)
        if idx is None or idx >= len(poses):
            return
        p = poses[idx]
        with self._gt_lock:
            first = self.gt_pose is None
            self.gt_pose = (p['x'], p['y'],
                            quat_to_yaw(p['qx'], p['qy'], p['qz'], p['qw']))
        if first:
            self.get_logger().info(
                f'真实位姿已生效: 实体="{p["name"]}" 下标={idx} '
                f'位置=({p["x"]:.3f}, {p["y"]:.3f})')

    def _resolve_ground_truth_index(self, poses):
        """确定 base_link 在 pose 列表中的下标。"""
        if self.gt_index >= 0:
            return self.gt_index

        # 优先按实体名匹配（"my_robot/base_link" 或以 /base_link 结尾）
        want = self.gt_model_prefix
        for i, p in enumerate(poses):
            if p['name'] == want or p['name'].endswith('/base_link'):
                self._gt_auto_index = i
                return i

        # 回退: 静态实体位姿恒定，只有机器人会动 -> 找第一个变动的下标
        snap = [(p['x'], p['y'], p['z']) for p in poses]
        if self._gt_first_sample is None:
            self._gt_first_sample = snap
            return None
        if len(self._gt_first_sample) != len(snap):
            self._gt_first_sample = snap
            return None
        for i, (cur, base) in enumerate(zip(snap, self._gt_first_sample)):
            if any(abs(a - b) > 1e-4 for a, b in zip(cur, base)):
                self._gt_auto_index = i
                self.get_logger().info(
                    f'自动探测到运动中实体的下标 = {i}（作为 base_link）')
                return i
        return self._gt_auto_index

    def check_pose_source(self):
        if self.pose_source in ('ground_truth', 'auto') and self.gt_pose is None:
            self.get_logger().warn(
                f'尚未取得真实位姿（{self.gt_gz_topic}）',
                throttle_duration_sec=30.0)

    # ==================================================================
    # 位姿来源: 轮式里程计 / 自积分
    # ==================================================================
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_pose = (p.x, p.y, quat_to_yaw(q.x, q.y, q.z, q.w))

    def on_cmd_vel(self, msg: Twist):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        self.last_cmd_time = now
        if dt <= 0.0 or dt > 1.0:
            return
        v, w = msg.linear.x, msg.angular.z
        if abs(w) < 1e-9:
            self.pose_x += v * math.cos(self.pose_yaw) * dt
            self.pose_y += v * math.sin(self.pose_yaw) * dt
        else:
            r = v / w
            self.pose_x += r * (math.sin(self.pose_yaw + w * dt) -
                                math.sin(self.pose_yaw))
            self.pose_y -= r * (math.cos(self.pose_yaw + w * dt) -
                                math.cos(self.pose_yaw))
            self.pose_yaw += w * dt
        self.pose_yaw = math.atan2(math.sin(self.pose_yaw),
                                   math.cos(self.pose_yaw))

    def current_pose(self):
        if self.pose_source == 'ground_truth':
            with self._gt_lock:
                if self.gt_pose is not None:
                    return self.gt_pose
        elif self.pose_source == 'odom':
            if self.odom_pose is not None:
                return self.odom_pose
        elif self.pose_source == 'integrated':
            return (self.pose_x, self.pose_y, self.pose_yaw)
        else:  # auto
            with self._gt_lock:
                if self.gt_pose is not None:
                    return self.gt_pose
            if self.odom_pose is not None:
                return self.odom_pose
        return (self.pose_x, self.pose_y, self.pose_yaw)

    # ==================================================================
    # TF
    # ==================================================================
    def publish_static_laser_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.frame_id
        t.transform.translation.x = self.mount_xyz[0]
        t.transform.translation.y = self.mount_xyz[1]
        t.transform.translation.z = self.mount_xyz[2]
        qx, qy, qz, qw = rpy_to_quat(*self.mount_rpy)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.static_tf.sendTransform(t)

    def on_tf_timer(self):
        x, y, yaw = self.current_pose()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = rpy_to_quat(0.0, 0.0, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    # ==================================================================
    # 扫描
    # ==================================================================
    def on_scan_timer(self):
        x, y, yaw = self.current_pose()
        ranges = raycast(x, y, yaw, self.beam_angles,
                         self.box_min, self.box_max)

        ranges = np.where((ranges >= self.range_min) & (ranges <= self.range_max),
                          ranges, np.inf)

        if self.noise_stddev > 0.0:
            valid = np.isfinite(ranges)
            if np.any(valid):
                noise = self.rng.normal(0.0, self.noise_stddev,
                                        size=int(valid.sum()))
                ranges[valid] = np.clip(ranges[valid] + noise,
                                        self.range_min, self.range_max)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = float(self.angle_min)
        msg.angle_max = float(self.angle_max)
        msg.angle_increment = float(self.angle_increment)
        msg.time_increment = 0.0
        msg.scan_time = float(1.0 / self.update_rate)
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = [float(r) for r in ranges]
        msg.intensities = [float(1.0 / r) if np.isfinite(r) and r > 0 else 0.0
                           for r in ranges]
        self.scan_pub.publish(msg)

    def destroy_node(self):
        self._gt_stop.set()
        if self._gt_proc is not None:
            try:
                self._gt_proc.terminate()
                self._gt_proc.wait(timeout=2)
            except Exception:
                try:
                    self._gt_proc.kill()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CpuLidarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
