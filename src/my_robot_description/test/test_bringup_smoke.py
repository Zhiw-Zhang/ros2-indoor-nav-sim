#!/usr/bin/env python3
"""bringup 冒烟测试：用**真实 launch 机制**验证 robot.launch.py 能起来。

它与 test_hil_end_to_end.py 的分工：

    test_hil_end_to_end.py   在一个进程里把驱动+虚拟车装到同一个 executor 上，
                             专测**行为与数值**（方向、尺度、超时、拒绝…）
    本文件                    起**真的进程**、真的走 `ros2 launch`，
                             专测**接线**：robot.launch.py 的参数、节点名、
                             话题名、TF 树、能不能被 ros2 CLI 看到

为什么不能只靠前者：前者是自己 new 出来的节点，绕过了 launch 系统。
而"launch 起不来 / 参数没传进去 / 可执行位不对 / install 里缺文件"这一类
问题**只有走真 launch 才会暴露**——本工程就真的踩过
（`install(PROGRAMS)` 装出来的文件没有可执行位，ros2 launch 直接报
 "executable not found"）。

跑法（需要创建 PTY）::

    python3 src/my_robot_description/test/test_bringup_smoke.py

退出码 0 = 通过。
"""

import os
import re
import shutil
import signal
import subprocess
import sys
import time

WS = '/home/zzw/sim_nav_ws'
SRC = os.path.join(WS, 'src', 'my_robot_description', 'src')


class Proc:
    """带进程组的子进程。用 start_new_session 保证能整组杀干净——
    ros2 launch 会派生一堆子进程，只杀父进程会留下孤儿节点占着话题。"""

    def __init__(self, cmd, log_path, env=None):
        self.log_path = log_path
        self.log = open(log_path, 'w')
        self.p = subprocess.Popen(
            cmd, stdout=self.log, stderr=subprocess.STDOUT,
            start_new_session=True, env=env,
            cwd=WS)

    def read(self):
        try:
            self.log.flush()
        except ValueError:
            pass          # 已经 close 了，下面的文件读取仍然可用
        try:
            with open(self.log_path) as f:
                return f.read()
        except OSError:
            return ''

    def kill(self):
        try:
            os.killpg(os.getpgid(self.p.pid), signal.SIGINT)
            self.p.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(self.p.pid), signal.SIGKILL)
            except Exception:
                pass
        try:
            self.log.close()
        except Exception:
            pass


def source_env():
    """把 install/setup.bash 的环境读进来（用 bash 展开，避免手拼 AMENT_PREFIX_PATH）。"""
    cmd = f'source /opt/ros/jazzy/setup.bash && source {WS}/install/setup.bash && env -0'
    out = subprocess.run(['bash', '-c', cmd], capture_output=True, check=True).stdout
    env = {}
    for chunk in out.split(b'\0'):
        if b'=' in chunk:
            k, v = chunk.split(b'=', 1)
            env[k.decode()] = v.decode()
    return env


class Results:
    def __init__(self):
        self.failed = []

    def check(self, name, ok, detail=''):
        print(f"  {'✓' if ok else '✗'} {name}" + (f"  {detail}" if detail else ''))
        if not ok:
            self.failed.append(name)


def run_ros(env, args, timeout=15):
    cmd = f'source {WS}/install/setup.bash && ' + ' '.join(args)
    try:
        r = subprocess.run(['bash', '-c', cmd], capture_output=True,
                           text=True, timeout=timeout, env=env, cwd=WS)
        return r.stdout + r.stderr
    except subprocess.TimeoutExpired as e:
        return (e.stdout or '') + (e.stderr or '')


def main():
    res = Results()
    print('=' * 72)
    print('bringup 冒烟测试：真实 ros2 launch + 虚拟 ACG720')
    print('=' * 72)

    # ---- 前置检查：install 里的文件与可执行位 ----
    print('\n[0] install 产物检查')
    libdir = os.path.join(WS, 'install/my_robot_description/lib/my_robot_description')
    for f in ('acg720_driver.py', 'acg720_simulator.py', 'acg720_protocol.py',
              'drive_kinematics.py'):
        p = os.path.join(libdir, f)
        exists = os.path.exists(p)
        executable = os.access(p, os.X_OK)
        # 驱动/模拟器是 ros2 run 的入口，必须可执行；协议/运动学只被 import
        need_x = f in ('acg720_driver.py', 'acg720_simulator.py')
        res.check(f'install 里有 {f}', exists, p)
        if need_x and exists:
            res.check(f'  {f} 可执行', executable,
                      '' if executable else
                      '★ install(PROGRAMS) 会照抄源码权限位，源码没 +x 就会失败')
    launch_p = os.path.join(
        WS, 'install/my_robot_description/share/my_robot_description/launch/robot.launch.py')
    res.check('install 里有 robot.launch.py', os.path.exists(launch_p))
    urdf_p = os.path.join(
        WS, 'install/my_robot_description/share/my_robot_description/urdf/acg720_real.urdf.xacro')
    res.check('install 里有 acg720_real.urdf.xacro', os.path.exists(urdf_p))
    if res.failed:
        print('\n前置检查未通过，先修这些再往下跑。')
        return 1

    env = source_env()

    # ---- 起虚拟车 ----
    print('\n[1] 启动虚拟 ACG720')
    sim = Proc([sys.executable, os.path.join(SRC, 'acg720_simulator.py'),
                '--no-driver'], '/tmp/smoke_sim.log', env=env)
    pty = None
    for _ in range(40):
        time.sleep(0.5)
        m = re.search(r'/dev/pts/\d+', sim.read())
        if m:
            pty = m.group(0)
            break
    res.check('虚拟车起来了并给出 PTY', pty is not None, str(pty))
    if not pty:
        print(sim.read()[-2000:])
        sim.kill()
        return 1

    # ---- 用 ros2 launch 起真机 bringup ----
    print(f'\n[2] ros2 launch robot.launch.py port:={pty} nav:=false')
    bot = Proc(['ros2', 'launch', 'my_robot_description', 'robot.launch.py',
                f'port:={pty}', 'nav:=false', 'rviz:=false'],
               '/tmp/smoke_bot.log', env=env)
    # 等节点起来。真机上这一段时间是"驱动打开串口 + 开始收遥测"。
    time.sleep(12)

    log = bot.read()
    res.check('launch 没有报错退出', bot.p.poll() is None,
              f'退出码 {bot.p.poll()}' if bot.p.poll() is not None else '')
    res.check('串口已打开', '串口已打开' in log,
              '' if '串口已打开' in log else log[-600:])
    res.check('没有 executable not found',
              'not found on the libexec' not in log)

    # ---- 话题与节点 ----
    print('\n[3] 话题与 TF')
    nodes = run_ros(env, ['ros2', 'node', 'list'], timeout=15)
    res.check('/acg720_driver 已启动', '/acg720_driver' in nodes, nodes.strip()[:200])
    res.check('/robot_state_publisher 已启动', '/robot_state_publisher' in nodes)

    topics = run_ros(env, ['ros2', 'topic', 'list', '-t'], timeout=15)
    res.check('/odom 存在', '/odom' in topics)
    res.check('/cmd_vel 存在', '/cmd_vel' in topics)

    odom = run_ros(env, ['ros2', 'topic', 'echo', '/odom', '--once'], timeout=20)
    res.check('/odom 有数据在发布', 'frame_id: odom' in odom,
              odom.strip().splitlines()[0] if odom.strip() else '无输出')
    res.check('odom child_frame_id = base_link', 'child_frame_id: base_link' in odom)

    # ★ TF 树必须完整：odom→base_link（驱动发）+ base_link→laser_frame（URDF 发）
    tfs = run_ros(env, ['ros2', 'topic', 'echo', '/tf', '--once'], timeout=20)
    tfs_static = run_ros(env, ['ros2', 'topic', 'echo', '/tf_static', '--once'],
                         timeout=20)
    res.check('odom → base_link 已发布', 'child_frame_id: base_link' in tfs, '')
    res.check('base_link → laser_frame 已发布',
              'laser_frame' in tfs_static,
              'URDF 的固定关节应当由 robot_state_publisher 发布')

    # ★ 关键：odom→base_link 只能有**一个**发布者（驱动）。
    #   如果 URDF 也发这条，就会 TF_REPEATED_DATA，RViz 里车会抖。
    info = run_ros(env, ['ros2', 'topic', 'info', '/tf'], timeout=15)
    m = re.search(r'Publisher count:\s*(\d+)', info)
    pub_count = int(m.group(1)) if m else -1
    res.check('/tf 发布者数量合理（<=2：驱动 + robot_state_publisher）',
              0 < pub_count <= 2, f'publisher count = {pub_count}')

    # ---- 遥控发速度，看轮子真的转 ----
    print('\n[4] 持续发 /cmd_vel，验证链路')
    # ★ 必须**连续**发。驱动的 cmd_vel_timeout 默认 0.5 s，超时就下发零速停车——
    #   这是真机的安全设计，不是 bug。用 `--once` 发一条再 sleep 1.5 s 去看里程计，
    #   量到的必然是"已经停了"的结果（本项目第一次就是这么写错的）。
    #   用 --rate 20 发 3 s，量到的才是"在走"的结果。
    def pos_x(txt):
        m = re.search(r'position:\s*\n\s*x:\s*([-\d.eE+]+)', txt)
        return float(m.group(1)) if m else None

    before = run_ros(env, ['ros2', 'topic', 'echo', '/odom', '--once'], timeout=20)
    x0 = pos_x(before)

    pub = Proc(['ros2', 'topic', 'pub', '--rate', '20', '/cmd_vel',
                'geometry_msgs/msg/Twist', '{linear: {x: 0.1}}'],
               '/tmp/smoke_pub.log', env=env)
    time.sleep(2.5)
    # 趁指令还在发的时候读诊断：诊断是 1 Hz 快照，且反映的是**当下**状态
    diag = run_ros(env, ['ros2', 'topic', 'echo', '/acg720/diagnostics', '--once'],
                   timeout=15)
    after = run_ros(env, ['ros2', 'topic', 'echo', '/odom', '--once'], timeout=20)
    pub.kill()
    x1 = pos_x(after)

    res.check('发出速度后里程计前进', x0 is not None and x1 is not None and x1 > x0 + 0.01,
              f'x: {x0} → {x1}')
    m = re.search(r'cmds=(\d+)', diag)
    res.check('驱动收到的指令数 > 0', bool(m) and int(m.group(1)) > 0,
              f'cmds={m.group(1)}' if m else '诊断里没有 cmds 字段')
    res.check('无 CRC 错误',
              'crc_errors=0' in diag,
              re.search(r'crc_errors=\d+', diag).group(0) if 'crc_errors=' in diag else '')
    res.check('诊断话题可用', 'connected=True' in diag, diag.strip()[:140])

    # ---- 停下来：cmd_vel 停发后必须自动停车 ----
    print('\n[5] 停发 /cmd_vel 后自动停车')
    time.sleep(1.2)                      # 超过 cmd_vel_timeout
    d1 = run_ros(env, ['ros2', 'topic', 'echo', '/odom', '--once'], timeout=20)
    time.sleep(1.2)
    d2 = run_ros(env, ['ros2', 'topic', 'echo', '/odom', '--once'], timeout=20)
    y1, y2 = pos_x(d1), pos_x(d2)
    res.check('超时后不再前进', y1 is not None and y2 is not None and abs(y2 - y1) < 0.005,
              f'x: {y1} → {y2}')

    # ---- 清理 ----
    bot.kill()
    sim.kill()
    time.sleep(1)

    print('\n' + '=' * 72)
    if res.failed:
        print(f'失败 {len(res.failed)} 项：')
        for i in res.failed:
            print(f'  - {i}')
        print('\n驱动日志尾部：')
        print('\n'.join(bot.read().splitlines()[-25:]))
        print('=' * 72)
        return 1
    print('全部通过 ✓')
    print('=' * 72)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
