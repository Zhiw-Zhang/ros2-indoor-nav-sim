# my_robot_lidar_sim

在 **WSL2** 等无法使用 Gazebo GPU 渲染传感器的环境下，为机器人提供可用的
2D 激光雷达 `/scan` 数据。

## 为什么需要这个包

Gazebo Sim 8 里**所有**激光雷达（`lidar` 与 `gpu_lidar`）都走渲染管线，
没有 CPU 实现。而 WSL2 上 Gazebo 的渲染传感器无法工作：

1. `type="lidar"`（旧版 Gazebo Classic 语法）已被 Gazebo Sim 8 弃用，
   服务器直接跳过该传感器：
   `[Wrn] [SdfEntityCreator.cc] Sensor type LIDAR not supported yet.`
   → 话题被创建但**永远没有发布者**。
2. 改用 `gpu_lidar` 并加载 `gz-sim-sensors-system` 后，渲染线程会永久卡死：
   `[Dbg] [Sensors.cc:337] Waiting for init`
   原因是 WSL2 的 EGL surfaceless 平台不可用
   （`libEGL: failed to get driver name for fd -1`、`Surfaceless platform:` 为空）。
   该阻塞会**拖死整个服务器主循环**，连 `/clock`、`/odom` 都停止发布。
3. 关键点：Sensors 系统**无论有没有渲染类传感器都会无条件初始化渲染线程**，
   因此只要加载了它，仿真就会卡死。

> 已实测无效的绕行方案：`--headless-rendering`、`EGL_PLATFORM=x11`、
> `LIBGL_ALWAYS_SOFTWARE=1` + llvmpipe、`ogre` 旧引擎、强制 GLX、带 GUI 运行。

## 方案

**保留 Gazebo 能正常工作的部分**（物理引擎、差速驱动、`/odom`、TF），
把不工作的雷达换成纯 CPU 光线投射节点：

| 组件 | 提供者 |
|---|---|
| 物理 / 动力学 / 碰撞 | Gazebo（world 中**不**加载 sensors 系统）|
| `/cmd_vel` → 运动 | `gz-sim-diff-drive-system` 插件 |
| `/odom`、`/tf`、`/clock` | `ros_gz_bridge` |
| 关节 TF | `robot_state_publisher` |
| **`/scan`** | **本包 `cpu_lidar_node`（CPU 光线投射，零 GPU）** |

## 位姿来源（容易踩的坑）

雷达必须知道自身位姿才能投射光束。**不能用 `/odom`**：

差速驱动的 `/odom` 是按轮子转速积分得到的。机器人撞墙被挡住后，
轮子仍在转（Gazebo 里会看到轮子持续打滑），于是 `/odom` 一路增长，
而机器人真实位置早就停了。实测：

| 时刻 | Gazebo 真实位置 | `/odom` |
|---|---|---|
| 撞墙后 | x = 2.692 m（停住） | x = 4.53 m |
| 继续给速度 | x = 2.692 m（没动） | x = 6.24 m |

若雷达用 `/odom`，它会以为自己在墙的另一侧，向前投射得到 `inf`，
**传感器会跟着里程计一起幻觉**。

因此本节点默认用 `pose_source: ground_truth`：直接读取 Gazebo 原生
`/world/default/pose/info` 真实位姿。撞墙后雷达仍准确报出 0.21 m 的
墙面距离（理论 0.2 m），而不受 `/odom` 漂移影响。

> 为什么不走 `ros_gz_bridge`：实测 `gz.msgs.Pose_V` →
> `geometry_msgs/msg/PoseArray` 这个转换不会真正转发数据
> （桥接日志显示创建成功，但 ROS 侧 `Publisher count: 0`）。
> 因此节点直接起 `gz topic -e` 子进程读取原生话题。
> 同理，`empty.sdf` 与 launch 中也**不要**再尝试桥接该类型。

可选值（`config/lidar_config.yaml` 的 `robot.pose_source`）：

* `ground_truth` —— 默认。读 Gazebo 真实位姿，无漂移。
* `odom` —— 用 `/odom`，会漂移（仅用于对比/教学演示）。
* `integrated` —— 完全自给自足，积分 `/cmd_vel`，不需要 Gazebo。
* `auto` —— 优先 `ground_truth`，取不到回退 `odom`。

## 用法

```bash
cd ~/sim_nav_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

ros2 launch my_robot_lidar_sim cpu_lidar_sim.launch.py
```

常用 launch 参数：

```bash
# 只跑 CPU 雷达，不启动 Gazebo（没有 /odom 时节点自行积分 cmd_vel 估计位姿）
ros2 launch my_robot_lidar_sim cpu_lidar_sim.launch.py use_gazebo:=false
```

驱动机器人：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

## 配置

编辑 `config/lidar_config.yaml`：

* `lidar.*` —— 雷达参数（帧名、频率、角度范围、光束数、量程、噪声、随机种子）
* `lidar.mount_xyz` —— 雷达安装位置，**必须与 URDF 中 `laser_joint` 的 origin 一致**
* `robot.pose_source` —— 位姿来源，见上文（默认 `ground_truth`）
* `robot.base_link_index` —— 默认 `-1`（自动探测）。节点优先按实体名
  `my_robot/base_link` 匹配；匹配不到则利用"world 中静态实体不动、
  只有机器人会动"的特性自动找出下标。若改了模型名可手动指定下标。
* `world.geometry` —— 障碍物矩形列表，**必须与 `worlds/empty.sdf` 保持一致**

修改 world 里的障碍物后，记得同步更新 `world.geometry`。

## 快速验证

```bash
# 独立验证：重新启动一次真实仿真，逐条打印 PASS/FAIL
bash src/my_robot_lidar_sim/scripts/verify.sh

# 带 RViz 可视化（红色点云应呈现墙与方块的轮廓）
bash src/my_robot_lidar_sim/scripts/verify.sh --rviz
```

`verify.sh` 中 CHK-1..CHK-7 应全部 PASS；CHK-8 是**故意设计为 FAIL**
的对照项，用来证明这套检查确实能检出失败（它验证 Gazebo 原生 GPU
雷达在 WSL2 下确实不工作）。

## 已验证

* `/scan` 速率 **10.01 Hz**（配置 10 Hz）
* 几何模块单元测试精确匹配理论值：正前方墙 2.9 m、
  侧向/后方 `inf`、box1 方向 2.1875 m
* 光线投射数学正确：静止时正前方读数 2.91 m（理论 2.9，含 stddev=0.01 噪声）
* TF 树完整：`odom → base_link → laser_frame`（高度 0.120 m）
* **撞墙场景**：`/odom` 已漂移到 6.24 m，雷达仍准确报出
  0.21 m 的墙面距离（理论 0.2 m）—— 证明位姿来源正确
* `/scan` 发布者恰好 1 个，无重复发布

## 回到原生 Linux 时

在具备正常 GPU 驱动的原生 Linux 上，可以改回 Gazebo 自带雷达：

1. 在 `worlds/empty.sdf` 中重新加入 sensors 系统：

   ```xml
   <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
     <render_engine>ogre2</render_engine>
   </plugin>
   ```

2. 启动 `my_robot_description` 的 `gazebo_sim.launch.py`，
   并在桥接参数中加回 `/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan`。

URDF 中已保留正确的 `type="gpu_lidar"` 传感器定义，无需改动。
