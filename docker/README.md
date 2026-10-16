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
docker/run.sh python3 -m pytest src/my_robot_description/test -q      # 349 例纯逻辑（板上 344 passed / 5 skipped）
docker/run.sh ros2 run my_robot_description platform_check.py
docker/run.sh ros2 run my_robot_description acg720_simulator.py --dry-run
docker/run.sh python3 src/my_robot_description/test/test_hil_end_to_end.py     # 59 项运动模式 HIL
docker/run.sh python3 src/my_robot_description/test/test_position_nav_hil.py   # 65 项 4A 分段导航 HIL
docker/run.sh python3 src/my_robot_description/test/test_bringup_smoke.py      # 真 launch，最慢（两分钟起）
```

`test_*.py` 里的 HIL/冒烟**不是 pytest 用例**（`conftest.py` 里已 `collect_ignore`），
要像上面那样单独 `python3` 跑；退出码 0 = 全过，项数由脚本自己打出来。

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
* **`ros2 topic pub` 默认不等订阅端匹配。** `--wait-matching-subscriptions`
  在不带 `--once/--times` 时默认是 **0**（`ros2topic/verb/pub.py` 里的
  `int(times != 0)`）—— 它打完 `beginning loop` 就立刻发，不管有没有人听。
  板子上 DDS 端点发现要一两秒，于是"发 1.2 s 速度看车动不动"这种写法会在
  板子上**静默失败**（开发机上却一直是绿的）。写脚本/排查时要么加 `-w 1`
  （`--max-wait-time-secs` 兜底），要么别用固定 sleep 卡窗口。
  本项目的冒烟测试就是这么翻的车，见 `test_bringup_smoke.py` 段 4 的注释。

---

## 接真底盘串口

仿真的 HIL 测试用的是容器**内部**的 PTY（`os.openpty()`），**不需要**真串口。
所以 `run.sh` 默认一个设备都不挂，**默认行为对仿真流程零影响**。

要接 ACG720 的真串口时，用环境变量显式指定设备节点：

```bash
ACG720_DEVICE=/dev/acg720 docker/run.sh ros2 run my_robot_description acg720_driver.py
```

`run.sh` 会自动做两件事：

1. 加 `--device <节点>`（只挂这一个设备，不是整个 `/dev`）；
2. **把设备节点的 gid `--group-add` 进容器**。

第 2 条不是可选项。`--user uid:gid` 只映射**主组**，宿主的**补充组不会
带进容器**；而 `/dev/ttyUSB*` 通常是 `root:dialout 0660`，宿主用户正是靠
`dialout` 这个补充组才读得到的。少了这一步，容器里打开设备会 `Permission
denied`，而报错发生在驱动内部，很容易被误判成"串口配置错了"。
（脚本是直接取 `stat -c %g` 的结果，不去猜组名 —— 换个发行版组名不一定
叫 `dialout`。）

### 先把设备名固定下来

别在命令里写 `/dev/ttyUSB0`：这个号会随插入顺序变。先确认识别情况：

```bash
ls -l /dev/ttyUSB*                          # 插上转接板后应当多出一个
udevadm info -a -n /dev/ttyUSB0 | grep -m2 -E 'idVendor|idProduct'
```

再写一条 udev 规则（`/etc/udev/rules.d/99-acg720.rules`），把芯片号换成
上面查到的值：

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  SYMLINK+="acg720", GROUP="dialout", MODE="0660"
```

然后 `sudo udevadm control --reload && sudo udevadm trigger`，确认
`/dev/acg720` 出现。

> ⚠️ **容器是在 `docker run` 那一刻拿设备节点的。USB 拔插之后必须重启容器**
> 才会重新拿到。不想重启就得 `-v /dev:/dev`，那等于把整个 `/dev` 交给容器，
> 代价远大于收益。所以规则要一次配对，别养成运行中拔插的习惯。

### 接线

取材自交接包的 `navigation/docs/HARDWARE.md`（**这个文件在交接包里，不在本
仓库**，所以这里直接抄下来，免得留一个点不开的路径）：

| 转接板 | FPGA | 说明 |
|---|---|---|
| USB-TTL **TX** | `E17`（`uart_rxd`） | 3.3 V 逻辑 |
| USB-TTL **RX** | `C20`（`uart_txd`） | |
| **GND** | FPGA `GND` | **必须共地**，否则字节会随机出错 |

⚠️ 电平必须是 **3.3 V**。很多 USB-TTL 板子有 5V/3.3V 跳线，务必确认在
3.3 V 档 —— FPGA 的 IO 是按 3.3 V 逻辑设计的。

> 顺带说明：这里走的是**外接 USB 转接板**，不是板子自己的调试串口，
> 所以 `console=ttyS2` 那条内核命令行参数**不冲突**，不用去动 `cmdline.txt`。

### 第一次接通时按这个顺序验

```bash
# 1) 宿主上确认设备在
ls -l /dev/acg720

# 2) 容器里能看到、能打开（不跑 ROS，先排除权限问题）
ACG720_DEVICE=/dev/acg720 docker/run.sh \
  python3 -c "import serial; s=serial.Serial('/dev/acg720',115200,timeout=1); print('打开成功', s.name); s.close()"

# 3) 起驱动，看位流版本号
ACG720_DEVICE=/dev/acg720 docker/run.sh \
  ros2 run my_robot_description acg720_driver.py --ros-args -p port:=/dev/acg720
```

第 3 步的日志里会出现 `位流版本校验通过：build_id=0x20260A02`。
**如果报"位流版本不符"，先别往下走** —— 多半是烧的是曲线模型那版位流，
这时驱动会拒绝发送任何运动命令（这是设计好的保护，不是故障）。

> 轮子上的测试务必**离地**。固件的 3.0 s 连续运动上限和 300 ms 命令超时
> 都会在台架上真实生效：任何一个没预期到就会锁存 FAULT，而 FAULT 是自锁的，
> 只能断电或「松开急停 S0 再按 S4」复位。
