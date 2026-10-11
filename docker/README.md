# 在 RK3588 板子上用容器跑 sim_nav_ws

## 为什么用容器

板子是 **Orange Pi 5 Plus**（aarch64），宿主系统是 **Ubuntu 22.04 Jammy**
（Orange Pi 官方 1.2.2 镜像，内核 5.10 Rockchip BSP）。

而 **ROS 2 Jazzy 只提供 Noble / 24.04 的二进制包**，22.04 上装不了。摆在面前
三条路：

| 方案 | 结论 |
|---|---|
| ① 刷 Ubuntu 24.04 | 长期最干净，但要重刷系统、重配网络，风险和工作量都大 |
| ② 宿主换 Humble | **否决**。Nav2 1.1.x 与 1.3.x 的参数键不一样，未声明的参数会让节点直接起不来 —— 等于永久分叉出一套真机配置，而仿真机必须留在 Jazzy |
| ③ **容器里跑 Jazzy** | **当前采用**。宿主一点都不动，容器里就是一个完整的 Noble + Jazzy |

容器这条路的前提已经验证过：`docker run --rm hello-world` 用 20.10 版
`docker.io` 跑通，`ros:jazzy-ros-base` 拉下来确认是 `linux/arm64`。

---

## 怎么用

```bash
cd ~/sim_nav_ws

# 1) 构建镜像（第一次，以及以后改了 docker/Dockerfile 之后）
docker/run.sh --build

# 2) 进容器（在 /ws 下）
docker/run.sh

# 3) 或者不进容器，直接在容器里跑一条命令
docker/run.sh colcon build --symlink-install
```

`run.sh` 会自动判断要不要套 `sudo`：加进 `docker` 组了就直接跑，没有就套
`sudo`（**不需要**把 `sudo` 写进命令里）。

### 容器里的验证顺序

从便宜到贵，一步步来，别一次全跑：

```bash
docker/run.sh colcon build --symlink-install
docker/run.sh ros2 run my_robot_description platform_check.py
docker/run.sh ros2 run my_robot_description acg720_simulator.py --dry-run
docker/run.sh python3 src/my_robot_description/test/test_hil_end_to_end.py
```

`platform_check.py` 会自己认出「我在容器里」：`/boot` 不存在、没有
`/dev/ttyS*` 这类在容器里必然的现象会被降级成提示，不算告警。

---

## 几个要记住的点

* **镜像里没有源码。** 仓库是运行时用 `-v $HOME/sim_nav_ws:/ws` 挂进去的，
  所以改代码**不用**重新 `--build`，直接重跑命令就行。
* **镜像名只写在 `run.sh` 的一行里**（`IMAGE=...`），Dockerfile 里不写。
* **`--user` 映射宿主用户**，所以 `build/ install/ log/` 不会被 root 占住。
  别手敲 `sudo docker run` 而不加 `--user`，那会生出一堆 root 所有的文件，
  之后普通用户 `colcon build` 就会 permission denied。
* **`--network host` + `--ipc host` 不能省。** host 网络是 DDS 能互相发现的前提；
  host IPC 是 FastDDS 共享内存传输的前提。
* **故意不装 rviz2。** 板子无头，可视化在别的机器上做。

---

## 以后要接底盘串口时

现在的 HIL 测试用的是容器**内部**的 PTY（`os.openpty()`），**不需要**真串口，
所以 `run.sh` 现在不挂 `/dev`、也不加 `--privileged` —— 这两样能不加就不加。

等真要接 ACG720 串口了，先确认芯片号：

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -m2 -E 'idVendor|idProduct'
```

然后写一条 udev 规则把设备名固定成 `/dev/acg720`（`SYMLINK+="acg720"`、
`GROUP="dialout"`、`MODE="0660"`），再在 `run.sh` 里加一行
`--device /dev/acg720`。

> 注意：容器是在启动那一刻拿设备节点的。**拔插 USB 之后必须重启容器**
> 才会重新拿到设备；不想重启就得 `--privileged -v /dev:/dev`，代价是把整个
> `/dev` 交给容器。所以规则要一次配对，别养成拔插的习惯。
