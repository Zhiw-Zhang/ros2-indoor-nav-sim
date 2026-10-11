# ============================================================================
#  sim_nav_ws 在 RK3588 板子上的运行镜像
#
#  为什么有这个文件：
#      板子是 Orange Pi 5 Plus，宿主系统 Ubuntu 22.04 Jammy，而 ROS 2 Jazzy
#      只出 Noble（24.04）的二进制包 —— 22.04 上装不了。所以把 Noble + Jazzy
#      整个放进容器，宿主一点都不用改。
#
#  构建 / 运行：都用同目录的 run.sh，别手敲 docker 命令。
#      docker/run.sh --build     构建 / 重建
#      docker/run.sh             进容器
#      详细说明见 docker/README.md
#
#  ── 三条设计约束（改这个文件前先读）──
#   1. 镜像里**不放源码**，运行时用 -v 把仓库挂进 /ws。仓库始终是唯一事实
#      来源：改代码不用重新 build 镜像，也不会出现「镜像里的代码是旧的」。
#   2. **不装 rviz2。** 板子是无头运行，可视化在别的机器上做。省下的空间和
#      依赖不是小数目。
#   3. ROS 发行版名「只在下行的 FROM 里出现一次」，下面的包名跟着它走。
#      要换发行版，先想清楚 Nav2 参数键在不同版本之间是不兼容的。
# ============================================================================
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ---- 运行依赖 ----
# 分组写清楚，以后好判断「这个包能不能删」：
#   python3-serial / python3-numpy / python3-pil
#       ← 驱动层直接 import（pyserial、numpy，PIL 给图像工具用）
#   python3-pytest / python3-colcon-common-extensions
#       ← 在板子上跑测试和构建
#   ros-jazzy-xacro / ros-jazzy-robot-state-publisher
#       ← robot.launch.py 要展开 acg720_real.urdf.xacro 并发布 TF
#   ros-jazzy-nav2-bringup
#       ← Nav2 全家桶：AMCL、MPPI、代价地图、collision_monitor、
#         velocity_smoother……都在里面
#   ros-jazzy-slam-toolbox
#       ← 建图，真机阶段要用
#   ros-jazzy-tf2-tools
#       ← 无头环境下查 TF 树（ros2 run tf2_tools view_frames）
#   ros-jazzy-teleop-twist-keyboard
#       ← 手推底盘，标定 wheel_separation_scale 时要靠它发 /cmd_vel
#
# 【待确认】激光雷达的驱动包**故意不在这里**：雷达型号还没定（LD19 /
#          RPLIDAR A1 这一档，具体未选）。定下来以后**单独加一层**，
#          不要动上面这层，否则每次换雷达都要把 Nav2 重装一遍。
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-serial \
        python3-numpy \
        python3-pil \
        python3-pytest \
        python3-colcon-common-extensions \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-tf2-tools \
        ros-jazzy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# 不设 ENTRYPOINT：run.sh 把要执行的命令直接接在镜像名后面，
# 临时换成 bash 排查问题时不用绕。
CMD ["bash"]
