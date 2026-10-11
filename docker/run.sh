#!/usr/bin/env bash
# ============================================================================
#  RK3588（Orange Pi 5 Plus）上跑 sim_nav_ws 的容器入口
#
#      docker/run.sh --build            构建 / 重建镜像
#      docker/run.sh                    进容器里的交互式 shell
#      docker/run.sh <命令> [参数...]    在容器里的 /ws 下执行命令
#
#  例：
#      docker/run.sh colcon build --symlink-install
#      docker/run.sh ros2 run my_robot_description platform_check.py
#
#  ── 五条设计约束（改这个脚本前先读）──
#   1. 镜像名 / 挂载点 / docker 参数**只写在这里**。别在命令行上再手敲一份
#      `docker run ...`，否则「容器到底怎么起」就有两个说法了。
#   2. 容器里的用户映射成宿主用户（--user）。这样 colcon build 出来的
#      build/ install/ log/ 归宿主用户所有，不用事后 sudo chown -R。
#   3. HOME 设成 /tmp。非 root 用户在镜像里写不动 /root，而 ROS 2 启动时
#      要建 $HOME/.ros/log；日志是一次性的，丢在 /tmp 正合适。
#   4. --network host 和 --ipc host 都是必须的：
#        host 网络 → DDS 才能和宿主上、乃至别的机器上的节点互相发现
#        host IPC  → FastDDS 的共享内存传输需要在 /dev/shm 上对齐
#   5. **不挂 /dev，也不加 --privileged。** 目前 HIL 测试用的是容器**内部**
#      的 PTY，不需要真串口。以后接底盘串口时再加 --device，并且要用 udev
#      把设备名固定住 —— 见 docker/README.md 末尾。
# ============================================================================
set -euo pipefail

# 镜像名只在这一行出现。
IMAGE="${ACG720_IMAGE:-sim-nav-ws:jazzy}"

# 宿主机上仓库的位置（挂到容器里的 /ws）。
WS="${ACG720_WS:-$HOME/sim_nav_ws}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 加进 docker 组了就直接跑，没有就套 sudo —— 两种都支持，不用改脚本。
if docker info >/dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi

if [ "${1:-}" = "--build" ]; then
  # 构建上下文就是本目录；镜像里不放源码，源码运行时靠 -v 挂进来。
  exec "${DOCKER[@]}" build -t "${IMAGE}" "${HERE}"
fi

# 宿主的 uid/gid 必须在 sudo **之前**取，否则取到的是 root 的 0:0。
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

# 没有终端时（被别的脚本调用、输出重定向到文件）不能加 -t，
# 否则 docker 会报 "the input device is not a TTY" 直接退出。
if [ -t 0 ] && [ -t 1 ]; then
  TTY=(-it)
else
  TTY=(-i)
fi

if ! "${DOCKER[@]}" image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "找不到镜像 ${IMAGE}，先构建一次：" >&2
  echo "    $0 --build" >&2
  exit 1
fi

exec "${DOCKER[@]}" run --rm "${TTY[@]}" \
  --network host \
  --ipc host \
  --user "${HOST_UID}:${HOST_GID}" \
  -e HOME=/tmp \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -v "${WS}:/ws" \
  -w /ws \
  "${IMAGE}" "$@"
