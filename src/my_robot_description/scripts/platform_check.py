#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""目标平台（RK3588）就绪自检 —— 只读诊断，不改动系统任何东西。

为什么需要它
============
本项目的驱动层在 x86 开发机上全绿（54 单测 + HIL + launch 冒烟），但真机跑在
RK3588（aarch64 / Ubuntu 24.04）上。换平台会踩的坑**都不是代码 bug**，而是
环境差异，具体是这几类：

  1. 发行版不对 -> ROS 2 Jazzy 的 aarch64 二进制包只给 Ubuntu 24.04 编译，
     22.04 上装不了。这是最容易翻车、也最贵的一条（要么刷机要么降级）。
  2. 缺 pyserial -> 驱动 `import serial` 直接崩，且开发机上通常早装了，
     本地怎么测都发现不了。
  3. 串口权限 -> 不在 dialout 组，打开串口 PermissionError。
  4. 串口控制台抢 UART -> Rockchip 镜像默认把内核控制台开在某一路 UART 上，
     正巧用了那一路就会看到日志往帧里灌。
  5. PTY 不可用 -> HIL 虚拟车跑不起来（容器/精简内核里会禁掉）。

用法
====
    python3 scripts/platform_check.py            # 人看的检查清单
    python3 scripts/platform_check.py -q         # 只打印有问题的项
    ros2 run my_robot_description platform_check.py

退出码：0 = 没有硬阻塞；1 = 有 ✗ 级别的阻塞项。

【待实测】标记的项在 x86 开发机上无法验证（比如控制台到底占哪一路 UART），
脚本只做**读取和提示**，判断留给人。
"""

from __future__ import annotations

import argparse
import glob
import os
import platform
import shutil
import subprocess
import sys

# ---- 期望值：目标平台是 RK3588 + Ubuntu 24.04 + ROS 2 Jazzy ----------------
EXPECT_ARCH = 'aarch64'
EXPECT_UBUNTU = '24.04'
EXPECT_ROS = 'jazzy'

OK = '  ✓'
BAD = '  ✗'
WARN = '  ⚠'
INFO = '  ·'

# 收集所有 ✗，最后决定退出码
_blockers: list[str] = []


def hard(msg: str) -> None:
    """记一条硬阻塞（✗）。"""
    _blockers.append(msg)


def section(title: str) -> None:
    print(f'\n=== {title} ===')


def read_text(path: str) -> str | None:
    """读文本，读不到就返回 None（不抛异常）。"""
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            return f.read()
    except OSError:
        return None


def in_container() -> bool:
    """是否跑在容器里（Docker / Podman）。

    为什么要在意：容器**共用宿主内核**，所以
      · /proc/cmdline 是宿主内核的（看 UART 控制台冲突仍然有意义）
      · /boot 通常没挂载 -> 第 5 节的启动配置检查会"什么都没找到"
      · /dev/ttyS* 是宿主的，可能一个都不透传进来
    这些都是容器里的正常现象，不是故障。不点明的话很容易被误读成
    "启动配置读不出来"然后去瞎折腾。
    """
    if os.path.exists('/.dockerenv') or os.path.exists('/run/.containerenv'):
        return True
    cgroup = read_text('/proc/1/cgroup') or ''
    return any(k in cgroup for k in ('docker', 'containerd', 'kubepods',
                                     'libpod'))


# ---------------------------------------------------------------- 1. 系统与架构
def check_os() -> None:
    section('1. 系统与架构')
    machine = platform.machine()
    print(f'{INFO} 架构: {machine}')
    if machine == EXPECT_ARCH:
        print(f'{OK} 架构是 {EXPECT_ARCH}')
    else:
        # 在开发机上跑必然走到这里，属于正常，不算阻塞
        print(f'{WARN} 架构不是 {EXPECT_ARCH} —— 现在大概是开发机（x86_64）。'
              f'本脚本的结论要在 RK3588 上跑才有效')

    rel = read_text('/etc/os-release') or ''
    kv = {}
    for line in rel.splitlines():
        if '=' in line:
            k, _, v = line.partition('=')
            kv[k.strip()] = v.strip().strip('"')
    pretty = kv.get('PRETTY_NAME', '(读不到 /etc/os-release)')
    ver = kv.get('VERSION_ID', '')
    print(f'{INFO} 发行版: {pretty}')

    if ver == EXPECT_UBUNTU:
        print(f'{OK} Ubuntu {ver}')
    elif ver:
        print(f'{BAD} Ubuntu {ver} ≠ 期望的 {EXPECT_UBUNTU}')
        print(f'     ROS 2 Jazzy 的 aarch64 官方 deb 是给 Ubuntu {EXPECT_UBUNTU} '
              f'(Noble) 编译的，{ver} 上装不了。')
        print(f'     出路：① 刷 Ubuntu {EXPECT_UBUNTU} 镜像（推荐）；'
              f'② 整个降到 Humble（本项目按 Jazzy 写的，要改一批配置）')
        hard(f'Ubuntu 版本 {ver} ≠ {EXPECT_UBUNTU}')
    else:
        print(f'{WARN} 读不到发行版版本号，无法判断能否装 Jazzy')

    print(f'{INFO} Python: {platform.python_version()} ({sys.executable})')

    if in_container():
        print(f'{INFO} ★ 运行在**容器**里。容器共用宿主内核，所以：')
        print(f'     · /proc/cmdline 是宿主内核的（看 UART 控制台冲突仍然有效）')
        print(f'     · /boot 一般没挂载 -> 第 5 节会"找不到启动配置"，属正常')
        print(f'     · /dev/ttyS* 是宿主的，没 --device 透传就看不到')


# ------------------------------------------------------------------- 2. ROS 2
def check_ros() -> None:
    section('2. ROS 2')
    distro = os.environ.get('ROS_DISTRO', '')
    ros2_bin = shutil.which('ros2')

    if not distro and not ros2_bin:
        print(f'{BAD} 没检测到 ROS 2（ROS_DISTRO 未设置，也找不到 ros2 命令）')
        print(f'     如果还没装，装法见 docs/rk3588_deploy.md')
        hard('ROS 2 未安装或未 source')
        return

    print(f'{INFO} ROS_DISTRO = {distro or "(未设置)"}')
    print(f'{INFO} ros2 命令: {ros2_bin or "(找不到)"}')

    if distro == EXPECT_ROS:
        print(f'{OK} ROS 2 {EXPECT_ROS}')
    elif distro:
        print(f'{BAD} ROS_DISTRO={distro} ≠ 期望的 {EXPECT_ROS}')
        print(f'     本项目的 launch / 参数文件是按 Jazzy 写的')
        hard(f'ROS 2 发行版 {distro} ≠ {EXPECT_ROS}')
    else:
        print(f'{WARN} ROS_DISTRO 未设置 —— 可能只是没 source 环境，'
              f'试试 `source /opt/ros/{EXPECT_ROS}/setup.bash`')

    # rclpy 能不能真 import（比看环境变量更可靠）
    try:
        import rclpy  # noqa: F401
        print(f'{OK} rclpy 可导入')
    except ImportError as e:
        print(f'{BAD} rclpy 导不进来: {e}')
        hard('rclpy 不可导入')


# ------------------------------------------------------- 3. Python 第三方依赖
def check_python_deps() -> None:
    section('3. Python 第三方依赖')
    # pyserial 是**硬依赖**：驱动和虚拟车都直接 import serial
    try:
        import serial
        from serial import VERSION as SERIAL_VERSION
        print(f'{OK} pyserial {SERIAL_VERSION}（驱动层硬依赖）')
    except ImportError:
        print(f'{BAD} 缺 pyserial —— acg720_driver.py 会直接崩在 import serial')
        print(f'     装法: sudo apt install python3-serial')
        print(f'     （package.xml 里已声明 python3-serial，'
              f'`rosdep install --from-paths src` 也会装它）')
        hard('缺 pyserial')

    # 下面两个只有仿真工具用，真机跑导航不需要，缺了只提示
    for mod, deb, why in (('numpy', 'python3-numpy', 'mapeval.py / cost_slice.py'),
                          ('PIL', 'python3-pil', 'mapeval.py / xwd2png.py')):
        try:
            __import__(mod)
            print(f'{OK} {mod}')
        except ImportError:
            print(f'{WARN} 缺 {mod}（只有 {why} 用；真机导航不需要）'
                  f' -> sudo apt install {deb}')


# ------------------------------------------------------------- 4. 串口与 PTY
def check_serial_and_pty() -> None:
    section('4. 串口与 PTY')

    # --- PTY：HIL 虚拟车靠它，没有它"无硬件验证"这条路就断了 ---
    try:
        import pty
        master, slave = pty.openpty()
        os.close(master)
        os.close(slave)
        print(f'{OK} PTY 可用（HIL 虚拟车能跑）')
    except OSError as e:
        print(f'{BAD} PTY 不可用: {e}')
        print(f'     影响: acg720_simulator.py 起不来 -> 没有硬件就无法验证驱动层')
        print(f'     可能是容器/精简内核禁了 devpts')
        hard('PTY 不可用')

    # --- 实际存在的串口设备 ---
    devs: list[str] = []
    for pat in ('/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyAMA*', '/dev/ttyS*'):
        devs.extend(sorted(glob.glob(pat)))
    devs = [d for d in devs if os.path.exists(d)]

    if devs:
        print(f'{INFO} 串口设备 {len(devs)} 个:')
        for d in devs:
            extra = ''
            try:
                st = os.stat(d)
                extra = f'  mode={oct(st.st_mode & 0o777)} gid={st.st_gid}'
                # 能打开才算真能用
                fd = os.open(d, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                os.close(fd)
                extra += ' 可打开'
            except OSError as e:
                extra += f' 打不开({e.strerror})'
            print(f'      {d}{extra}')
        print(f'{INFO} 真机用的是 USB 转串口（ttyUSB*/ttyACM*）还是板载 UART（ttyS*），')
        print(f'     取决于你实际怎么接线；ACG720 那侧波特率见 docs/real_robot_params.md')
    else:
        if in_container():
            print(f'{INFO} 容器里没看到串口设备 —— 这是正常的。'
                  f'接了硬件后要用 --device=/dev/ttyXXX 透传进来')
        else:
            print(f'{WARN} 没发现任何串口设备 —— 小车还没接？'
                  f'（接了的话插拔一下看 dmesg | tail）')

    # --- dialout 组：不在里面就 open() 不了串口 ---
    import grp
    try:
        gid = grp.getgrnam('dialout').gr_gid
    except KeyError:
        gid = None
    user = os.environ.get('USER') or os.environ.get('LOGNAME') or '(未知)'
    groups = os.getgroups()
    if os.geteuid() == 0:
        # ★ root 不受组权限限制，能直接打开串口。容器里默认就是 root，
        #   如果这里按普通用户那样判，会误报一条根本不存在的硬阻塞 ——
        #   而我们推荐的第一条路恰好就是"在容器里跑"，所以必须区分开。
        print(f'{OK} 当前是 root（uid 0），不受 dialout 组限制，可直接打开串口')
        print(f'     （若改用普通用户跑，需把它加进 dialout 组）')
    elif gid is not None and gid in groups:
        print(f'{OK} 当前用户 {user} 在 dialout 组')
    elif gid is not None:
        print(f'{BAD} 当前用户 {user} **不在** dialout 组 —— 打开串口会 PermissionError')
        print(f'     修: sudo usermod -aG dialout {user}  然后**重新登录**')
        print(f'     （重登才生效；临时验证可以用 newgrp dialout 开个子 shell）')
        hard(f'{user} 不在 dialout 组')
    else:
        print(f'{WARN} 系统里没有 dialout 组，串口权限模型可能不同')


# ------------------------------------------- 5. 启动配置与串口控制台冲突
def check_boot_and_console() -> None:
    section('5. 启动配置与串口控制台冲突')
    print(f'{INFO} 目的：Rockchip 镜像默认把内核控制台开在某一路上，')
    print(f'     如果正好用了那一路 UART，内核日志会混进协议帧流。')

    # 当前内核命令行（最权威）
    cmdline = read_text('/proc/cmdline')
    if cmdline is not None:
        print(f'{INFO} /proc/cmdline:')
        print(f'      {cmdline.strip()}')
        # 提取 console=ttyXXX
        consoles = [tok for tok in cmdline.split() if tok.startswith('console=')]
        if consoles:
            print(f'{WARN} 内核控制台正在使用: {", ".join(consoles)}')
            print(f'     如果 ACG720 接的正好是这一路，必须关掉它（见下）')
        else:
            print(f'{OK} 内核命令行里没有 console= 项')

    # 各种发行版/BSP 的启动配置文件，存在哪个算哪个
    candidates = [
        '/boot/extlinux/extlinux.conf',   # Rockchip BSP / Orange Pi 官方镜像
        '/boot/orangepiEnv.txt',          # Orange Pi 部分镜像
        '/boot/armbianEnv.txt',           # Armbian
        '/boot/firmware/cmdline.txt',     # Ubuntu 标准
        '/boot/uEnv.txt',
    ]
    found = [p for p in candidates if os.path.isfile(p)]
    if not found:
        if in_container():
            print(f'{INFO} 容器里看不到宿主 /boot，这一项**跳过**（去宿主上查）')
        else:
            print(f'{WARN} 没找到已知的启动配置文件（可能路径不同/权限不足）')
    for p in found:
        txt = read_text(p)
        if txt is None:
            print(f'{WARN} {p} 存在但读不了（需要 sudo）')
            continue
        print(f'{INFO} 发现启动配置: {p}')
        for line in txt.splitlines():
            s = line.strip()
            if s.startswith('#'):
                continue
            if 'console=' in s or 'overlays=' in s or 'overlay=' in s:
                print(f'        {s}')

    print(f'{INFO} 关闭控制台的通用做法（【待实测】具体哪一路要按上面输出判断）:')
    print(f'     Orange Pi 官方镜像 : 编辑 /boot/extlinux/extlinux.conf，'
          f'去掉内核行里的 console=ttyS2,1500000')
    print(f'     Armbian            : /boot/armbianEnv.txt 里去掉 console= 相关项，'
          f'再 sudo systemctl disable serial-getty@ttyS2')
    print(f'     别忘了 getty       : systemctl status serial-getty@ttyS2')

    # 板载 UART 的 overlay 开关（Orange Pi 走 overlay 打开排针上的 UART）
    print(f'{INFO} Orange Pi 5 系列排针上的 UART 通常要靠 overlay 打开，'
          f'【待实测】具体名字以板子手册为准')


# ---------------------------------------------------------------- 6. CPU/内存
def check_hardware() -> None:
    section('6. CPU 与内存')
    cpuinfo = read_text('/proc/cpuinfo') or ''
    # 按优先级挑型号字段。注意不能简单用 startswith('model')：
    # x86 的 /proc/cpuinfo 里 `model : 183`（纯数字）排在 `model name : ...`
    # **前面**，按前缀命中的话型号会显示成 "183"。
    fields: dict[str, str] = {}
    for line in cpuinfo.splitlines():
        k, sep, v = line.partition(':')
        if sep:
            fields.setdefault(k.strip().lower(), v.strip())
    model = ''
    for key in ('model name', 'hardware', 'model'):
        if fields.get(key):
            model = fields[key]
            break
    n = os.cpu_count()
    print(f'{INFO} CPU: {model or "(未知)"}')
    print(f'{INFO} 逻辑核数: {n}')

    mem_gb = None
    for line in (read_text('/proc/meminfo') or '').splitlines():
        if line.startswith('MemTotal:'):
            try:
                mem_gb = int(line.split()[1]) / 1024 / 1024
            except (IndexError, ValueError):
                pass
            break
    if mem_gb:
        print(f'{INFO} 内存: {mem_gb:.1f} GiB')
        if mem_gb < 3.5:
            print(f'{WARN} 内存偏小，Nav2 全局代价地图 + AMCL 会比较紧张')
    if n and n < 4:
        print(f'{WARN} 核数偏少，Nav2 的 controller_frequency 建议降到 10~15 Hz')

    load = os.getloadavg()
    print(f'{INFO} 当前负载(1/5/15min): {load[0]:.2f} / {load[1]:.2f} / {load[2]:.2f}')

    # 提醒别在板上跑 Gazebo
    print(f'{INFO} 提醒：真机不需要 Gazebo。仿真留在开发机，'
          f'板子上只跑运行时节点')


# -------------------------------------------------------------------- 7. 网络
def check_network() -> None:
    section('7. 网络与跨机 DDS')
    # 列出非 lo 的 IPv4
    addrs: list[str] = []
    try:
        out = subprocess.run(['ip', '-4', '-o', 'addr'], capture_output=True,
                             text=True, timeout=5).stdout
        for line in out.splitlines():
            parts = line.split()
            if len(parts) >= 4 and parts[1] != 'lo':
                addrs.append(f'{parts[1]} {parts[3]}')
    except (OSError, subprocess.SubprocessError):
        pass
    if addrs:
        for a in addrs:
            print(f'{INFO} 网卡 {a}')
    else:
        print(f'{WARN} 没读到非 lo 的 IPv4（ip 命令不可用？还是没联网？）')
        print(f'     板子必须和开发机在同一网段，RViz 才能跨机看到数据')

    for var in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION',
                'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_LOCALHOST_ONLY'):
        print(f'{INFO} {var} = {os.environ.get(var, "(未设置)")}')
    print(f'{INFO} 跨机看 RViz 要求两台机器 ROS_DOMAIN_ID 一致；'
          f'有线直连比 WiFi 稳（WiFi 可能挡多播）')
    print(f'{INFO} ROS_LOCALHOST_ONLY 若为 1，跨机通信会被完全切断 —— 必须是 0/未设置')


# ---------------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser(
        description='目标平台（RK3588）就绪自检，只读，不改动系统')
    ap.add_argument('-q', '--quiet', action='store_true',
                    help='只打印有问题（✗/⚠）的项')
    args = ap.parse_args()

    print('=' * 68)
    print('目标平台就绪自检  ——  RK3588 / Ubuntu 24.04 / ROS 2 Jazzy')
    print('=' * 68)

    # 各检查函数自己控制输出；-q 时把 stdout 吞掉再选择性重放太绕，
    # 所以这里简单处理：-q 就不逐段打印，只在最后汇总阻塞项。
    if args.quiet:
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            check_os()
            check_ros()
            check_python_deps()
            check_serial_and_pty()
            check_boot_and_console()
            check_hardware()
            check_network()
        for line in buf.getvalue().splitlines():
            if line.startswith((BAD, WARN)) or line.startswith('==='):
                print(line)
    else:
        check_os()
        check_ros()
        check_python_deps()
        check_serial_and_pty()
        check_boot_and_console()
        check_hardware()
        check_network()

    print('\n' + '=' * 68)
    if _blockers:
        print(f'有 {len(_blockers)} 项硬阻塞，先解决这些再往下走：')
        for i, b in enumerate(_blockers, 1):
            print(f'  {i}. {b}')
        print('=' * 68)
        return 1
    print('没有硬阻塞 ✓')
    print('下一步：colcon build --symlink-install，然后跑 HIL 虚拟车')
    print('        python3 src/my_robot_description/src/acg720_simulator.py --dry-run')
    print('=' * 68)
    return 0


if __name__ == '__main__':
    sys.exit(main())
