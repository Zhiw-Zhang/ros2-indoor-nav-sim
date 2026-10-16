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
#   5. **默认不挂 /dev，也不加 --privileged。** 仿真的 HIL 测试用的是容器
#      **内部**的 PTY，不需要真串口，所以默认什么都不挂 —— 能不加就不加。
#      要接真底盘时用 `ACG720_DEVICE=/dev/xxx` 显式指定（见下面的 DEV_ARGS）。
#      **不要**为了图省事改成 `-v /dev:/dev` 或 `--privileged`：
#      那等于把整个 /dev 交给容器，代价远大于收益。详见 docker/README.md 末尾。
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

# ── 真串口（可选）────────────────────────────────────────────────────────
# 默认一个设备都不挂。要接 ACG720 底盘时：
#
#     ACG720_DEVICE=/dev/acg720 docker/run.sh ros2 run my_robot_description acg720_driver.py
#
# 推荐先用 udev 把设备名固定住（见 docker/README.md 末尾），别直接写
# /dev/ttyUSB0 —— 那个号会随插入顺序变，而且等到两条规则打架时才知道疼。
# 这段放在镜像检查**之前**：参数写错要立刻报错，别让人先等镜像检查。
DEV_ARGS=()
if [ -n "${ACG720_DEVICE:-}" ]; then
  if [ ! -e "${ACG720_DEVICE}" ]; then
    echo "ACG720_DEVICE=${ACG720_DEVICE} 不存在（还没插，或者名字不对）。" >&2
    echo "当前宿主上的串口设备：" >&2
    ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null >&2 || echo "    （一个都没有）" >&2
    exit 1
  fi
  DEV_ARGS+=(--device "${ACG720_DEVICE}")

  # ★ 光有 --device 是不够的。`--user uid:gid` 只映射**主**组，宿主的
  #   **补充组不会带进容器**。而 /dev/ttyUSB* 通常是 root:dialout 0660，
  #   宿主用户正是靠 dialout 这个补充组才读得到的 —— 容器里没有这个组，
  #   打开设备就会 Permission denied（而且报错发生在驱动里，很容易误判成
  #   "串口配置错了"）。这里把设备节点的 gid 直接 --group-add 进去，
  #   不去猜组名（组名在别的发行版上不一定叫 dialout）。
  DEV_GID="$(stat -c '%g' "${ACG720_DEVICE}")"
  if [ "${DEV_GID}" != "0" ]; then
    DEV_ARGS+=(--group-add "${DEV_GID}")
  fi
fi

if ! "${DOCKER[@]}" image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "找不到镜像 ${IMAGE}，先构建一次：" >&2
  echo "    $0 --build" >&2
  exit 1
fi

# ⚠️ 设备节点是在 `docker run` 那一刻拿到的：**USB 拔插之后必须重启容器**。
#    想避免这点就得 `-v /dev:/dev`，代价见文件头第 5 条。所以请把 udev 规则
#    一次配对，别养成运行中拔插的习惯。

exec "${DOCKER[@]}" run --rm "${TTY[@]}" \
  --network host \
  --ipc host \
  --user "${HOST_UID}:${HOST_GID}" \
  "${DEV_ARGS[@]}" \
  -e HOME=/tmp \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -v "${WS}:/ws" \
  -w /ws \
  "${IMAGE}" "$@"
