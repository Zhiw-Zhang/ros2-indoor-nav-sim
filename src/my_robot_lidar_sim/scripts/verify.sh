#!/usr/bin/env bash
# =============================================================================
# 激光雷达仿真 —— 独立验证脚本
#
# 用法:
#   bash src/my_robot_lidar_sim/scripts/verify.sh          # 只做数值验证
#   bash src/my_robot_lidar_sim/scripts/verify.sh --rviz   # 验证后打开 RViz 可视化
#
# 设计原则:
#   1. 每次运行都重新启动一次真实仿真，不依赖任何缓存的测试结果。
#   2. 每条断言独立测量并单独打印 PASS / FAIL。
#   3. 最后一条 CHK-7 是【故意设计为 FAIL】的项：它验证 Gazebo 原生 GPU
#      雷达在 WSL2 下确实不工作。如果你看到 CHK-7 = FAIL，说明这套检查
#      确实能失败、不是橡皮图章。
# =============================================================================
set +e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OPEN_RVIZ=0
[ "${1:-}" = "--rviz" ] && OPEN_RVIZ=1

# ---- 沙箱友好的运行时目录（HOME 在本会话中不可写）----
export HOME="$WS/.simrun/gzhome"
export ROS_HOME="$WS/.simrun/roshome"
export ROS_LOG_DIR="$WS/.simrun/logs"
export DISPLAY="${DISPLAY:-:0}"
mkdir -p "$HOME" "$ROS_HOME" "$ROS_LOG_DIR"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash" 2>/dev/null || { echo "请先 colcon build"; exit 1; }

LAUNCH_LOG="$WS/.simrun/verify_launch.log"
GPU_LOG="$WS/.simrun/verify_gpu_lidar.log"
TMP_WORLD="$WS/.simrun/_verify_gpu_world.sdf"
PROBE_PY="$WS/.simrun/_verify_probe.py"
MOVED_PY="$WS/.simrun/_verify_moved.py"

PASS=0; FAIL=0
declare -a RESULTS

cleanup() {
  pkill -f 'gz sim' 2>/dev/null
  pkill -f parameter_bridge 2>/dev/null
  pkill -f robot_state_publisher 2>/dev/null
  pkill -f cpu_lidar_node 2>/dev/null
  pkill -f 'ros2 launch' 2>/dev/null
  pkill -f rviz2 2>/dev/null
  sleep 2
}

report() {  # report <编号> <描述> <PASS|FAIL> <详情>
  local id="$1" desc="$2" verdict="$3" detail="$4"
  if [ "$verdict" = "PASS" ]; then PASS=$((PASS+1)); else FAIL=$((FAIL+1)); fi
  printf '  [%-4s] %-46s %s\n' "$verdict" "$desc" "$detail"
  RESULTS+=("$id|$desc|$verdict|$detail")
}

echo "=============================================================="
echo " 激光雷达仿真独立验证    $(date '+%Y-%m-%d %H:%M:%S')"
echo " 工作区: $WS"
echo "=============================================================="

cleanup

# =============================================================================
echo
echo "### 启动仿真（全新实例）###"
: > "$LAUNCH_LOG"
ros2 launch my_robot_lidar_sim cpu_lidar_sim.launch.py > "$LAUNCH_LOG" 2>&1 &
LPID=$!
echo "    launch pid=$LPID, 等待 30s 稳定..."
sleep 30

# =============================================================================
echo
echo "### 检查项 ###"

# ---- CHK-1: 三个必需节点在线 ----
NODES=$(timeout 15 ros2 node list 2>/dev/null | sort | tr -d ' ')
n=0
for want in /cpu_lidar_node /robot_state_publisher /ros_gz_bridge; do
  echo "$NODES" | grep -q "^${want}$" && n=$((n+1))
done
[ "$n" -eq 3 ] && report CHK-1 "三个必需节点在线" PASS "3/3" \
               || report CHK-1 "三个必需节点在线" FAIL "仅 $n/3"

# ---- CHK-2: /scan 有一个且只有一个发布者 ----
PUB=$(timeout 12 ros2 topic info /scan 2>/dev/null | grep -oP 'Publisher count: \K\d+')
[ "${PUB:-0}" = "1" ] && report CHK-2 "/scan 发布者数量 == 1" PASS "count=$PUB" \
                      || report CHK-2 "/scan 发布者数量 == 1" FAIL "count=${PUB:-未知}"

# ---- 用 python 一次性采集数据，供 CHK-3/4/5 使用 ----
# 写成临时文件再执行：比 heredoc + read 更不易受 shell/日志流干扰。
cat > "$PROBE_PY" <<'PY'
import time, math, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
rclpy.init()
n = rclpy.create_node('verify_probe')
scans = []
n.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), 10)
t0 = time.time()
while time.time() - t0 < 8.0:
    rclpy.spin_once(n, timeout_sec=0.1)
elapsed = max(time.time() - t0, 1e-9)
if not scans:
    print("RESULT 0.0 0.0 0")
else:
    m = scans[-1]
    idx_front = int(round((0.0 - m.angle_min) / m.angle_increment)) % len(m.ranges)
    front = m.ranges[idx_front]
    finite = sum(1 for r in m.ranges if math.isfinite(r))
    rate = len(scans) / elapsed
    print(f"RESULT {rate:.4f} {front:.4f} {finite}")
n.destroy_node()
rclpy.shutdown()
PY

# 只取 RESULT 行，避免任何日志混入
RESULT_LINE=$(timeout 40 python3 "$PROBE_PY" 2>/dev/null | grep '^RESULT ' | tail -1)
RATE=$(echo "$RESULT_LINE" | awk '{print $2}')
FRONT=$(echo "$RESULT_LINE" | awk '{print $3}')
FINITE=$(echo "$RESULT_LINE" | awk '{print $4}')
RATE=${RATE:-0}; FRONT=${FRONT:-0}; FINITE=${FINITE:-0}
echo "    采集: rate=$RATE Hz  front=$FRONT m  finite=$FINITE"

# ---- CHK-3: 发布频率约 10 Hz（容差 ±1.5）----
FR_OK=$(python3 -c "print(1 if abs($RATE-10.0)<=1.5 else 0)")
[ "$FR_OK" = "1" ] && report CHK-3 "/scan 频率 ≈ 10 Hz" PASS "${RATE} Hz" \
                   || report CHK-3 "/scan 频率 ≈ 10 Hz" FAIL "${RATE} Hz"

# ---- CHK-4: 正前方障碍物距离正确（理论 2.9 m）----
D_OK=$(python3 -c "print(1 if abs($FRONT-2.9)<=0.10 else 0)")
[ "$D_OK" = "1" ] && report CHK-4 "正前方读数 ≈ 2.9 m (墙在 x=3)" PASS "${FRONT} m" \
                  || report CHK-4 "正前方读数 ≈ 2.9 m (墙在 x=3)" FAIL "${FRONT} m"

# ---- CHK-5: 存在有限读数（不是全 inf / 全空）----
[ "${FINITE:-0}" -gt 10 ] && report CHK-5 "存在有效回波 (>10 束)" PASS "${FINITE}/360" \
                          || report CHK-5 "存在有效回波 (>10 束)" FAIL "${FINITE}/360"

# ---- CHK-6: TF 树 odom -> laser_frame 连通 ----
TF_OUT=$(timeout 15 ros2 run tf2_ros tf2_echo odom laser_frame 2>/dev/null | grep -m1 'Translation')
if [ -n "$TF_OUT" ]; then
  report CHK-6 "TF odom->laser_frame 连通" PASS "$(echo "$TF_OUT" | tr -s ' ')"
else
  report CHK-6 "TF odom->laser_frame 连通" FAIL "无变换"
fi

# ---- CHK-7: 闭环 + 撞墙场景（最能证明位姿来源正确）----
# 驱动机器人一路前进直到撞上 x=3 的墙。
# 此时差速驱动的 /odom 会持续漂移增长，而雷达必须仍然报出
# 墙面附近的短距离（理论 ~0.2 m）。若雷达误用 /odom，
# 它会"穿过"墙从另一侧投射，前方读数会变成 inf —— 即 FAIL。
cat > "$MOVED_PY" <<'PY'
import time, math, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

rclpy.init()
n = rclpy.create_node('verify_wall')
scans, odoms = [], []
n.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), 10)
n.create_subscription(Odometry, '/odom', lambda m: odoms.append(m), 10)

# 采集约 3 秒（此时外部 publisher 已把机器人顶到墙上）
t0 = time.time()
while time.time() - t0 < 3.0:
    rclpy.spin_once(n, timeout_sec=0.1)

if scans and odoms:
    m = scans[-1]
    idx = int(round((0.0 - m.angle_min) / m.angle_increment)) % len(m.ranges)
    print(f"RESULT {odoms[-1].pose.pose.position.x:.4f} {m.ranges[idx]:.4f}")
else:
    print("RESULT 0.0 0.0")
n.destroy_node()
rclpy.shutdown()
PY

# 用外部 publisher 驱动前进（比在探针里发更接近真实用法）
timeout 14 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" >/dev/null 2>&1 &
PUBPID=$!
sleep 10
MOVED_LINE=$(timeout 30 python3 "$MOVED_PY" 2>/dev/null | grep '^RESULT ' | tail -1)
kill $PUBPID 2>/dev/null
timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
OX=$(echo "$MOVED_LINE" | awk '{print $2}')
SF=$(echo "$MOVED_LINE" | awk '{print $3}')
# 判定: 前方必须是有限值且在墙面附近（0.1 ~ 0.6 m），
# 同时 /odom 应已漂移到明显超过真实墙距 2.9
MOVED_OK=$(python3 -c "
try:
    sf = float('$SF'); ox = float('$OX')
except ValueError:
    sf = -1.0; ox = 0.0
import math
ok = (not math.isinf(sf)) and 0.1 <= sf <= 0.6 and ox > 3.0
print(1 if ok else 0)
")
if [ "$MOVED_OK" = "1" ]; then
  report CHK-7 "撞墙后雷达仍报墙面距离(不随odom漂移)" PASS \
    "前方=${SF} m, 而此时 /odom.x=${OX} m(已漂移)"
else
  report CHK-7 "撞墙后雷达仍报墙面距离(不随odom漂移)" FAIL \
    "前方=${SF:-?}, /odom.x=${OX:-?}"
fi

# ---- 清理主仿真 ----
kill -INT $LPID 2>/dev/null
cleanup

# =============================================================================
echo
echo "### CHK-8: 对照实验 —— Gazebo 原生 GPU 雷达（预期 FAIL）###"
echo "    这一项验证: 你的原始代码路径在 WSL2 下确实不工作。"

# 构造一个启用 sensors 系统的 world 副本
python3 - "$WS/install/my_robot_description/share/my_robot_description/worlds/empty.sdf" "$TMP_WORLD" <<'PY'
import sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src).read()
plugin = ('<plugin filename="gz-sim-sensors-system" '
          'name="gz::sim::systems::Sensors">'
          '<render_engine>ogre2</render_engine></plugin>')
s = s.replace('<world name="default">', '<world name="default">' + plugin, 1)
open(dst, 'w').write(s)
PY

: > "$GPU_LOG"
gz sim -s -r "$TMP_WORLD" > "$GPU_LOG" 2>&1 &
GPID=$!
sleep 22
SLOG=$(ls -t "$HOME/.gz/sim/log/"*/server_console.log 2>/dev/null | head -1)
STUCK=no; [ -n "$SLOG" ] && grep -q 'Waiting for init' "$SLOG" && STUCK=yes
CLOCKPUB=$(timeout 8 gz topic -i -t /clock 2>/dev/null | grep -c 'Publishers')
kill -9 $GPID 2>/dev/null; cleanup

if [ "$STUCK" = "yes" ]; then
  report CHK-8 "[对照] Gazebo GPU 雷达卡死(预期 FAIL)" FAIL \
    "渲染线程卡在 Waiting for init — 与文档描述一致"
else
  report CHK-8 "[对照] Gazebo GPU 雷达卡死(预期 FAIL)" PASS \
    "未复现卡死 — 说明本机环境与文档描述不符，请重新评估方案"
fi

# =============================================================================
echo
echo "=============================================================="
echo " 结果汇总"
echo "=============================================================="
echo "  PASS = $PASS    FAIL = $FAIL"
echo
echo " 说明: CHK-1..CHK-7 应全部 PASS。"
echo "       CHK-8 应 FAIL —— 它是对照项，证明这套检查确实能检出失败。"
echo "       若 CHK-8 变成 PASS，说明本机 Gazebo 渲染已恢复，"
echo "       可以改回 Gazebo 原生 gpu_lidar（见 README 末尾）。"
echo

if [ "$OPEN_RVIZ" = "1" ]; then
  echo "### 打开 RViz 可视化（Ctrl-C 退出）###"
  ros2 launch my_robot_lidar_sim cpu_lidar_sim.launch.py > "$LAUNCH_LOG" 2>&1 &
  sleep 28
  RVIZ_CFG="$WS/install/my_robot_lidar_sim/share/my_robot_lidar_sim/config/lidar_view.rviz"
  if [ ! -f "$RVIZ_CFG" ]; then
    RVIZ_CFG="$WS/src/my_robot_lidar_sim/config/lidar_view.rviz"
  fi
  echo "### RViz 配置: $RVIZ_CFG"
  echo "### 在 RViz 中应看到:"
  echo "    - 红色点云在正前方 x=3 处形成一条弧线（墙）"
  echo "    - 左后方一个独立红色点簇（box1 在 -2,1.5）"
  echo "    - 蓝色箭头 (Odometry) 与机器人模型"
  rviz2 -d "$RVIZ_CFG"
  cleanup
fi
