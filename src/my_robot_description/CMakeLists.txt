cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch worlds config maps routes docs
  DESTINATION share/${PROJECT_NAME}
)

# 工具脚本，装到 lib/${PROJECT_NAME} 下，用 `ros2 run` 调用：
#     ros2 run my_robot_description mapeval.py maps/rooms.yaml
#     ros2 run my_robot_description nav_goals.py -2.5,1.9 2.5,1.9
#     ros2 run my_robot_description cost_slice.py 4.0,4.95,-2.4,-0.85
#     ros2 run my_robot_description xwd2png.py in.xwd out.png
# 用 install(PROGRAMS ...) 而不是 install(FILES ...)，前者会带上可执行位。
install(PROGRAMS
  scripts/mapeval.py
  scripts/obs_test.py
  scripts/nav_goals.py
  scripts/cost_slice.py
  scripts/xwd2png.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- ACG720 真机驱动层 ----
# 串口协议 + 底盘驱动 + 虚拟底盘。这些是**真机**用的，仿真流程不受影响。
#     ros2 run my_robot_description acg720_driver      真机底盘驱动
#     ros2 run my_robot_description acg720_simulator   没有硬件时的虚拟底盘
# 协议与运动学是纯 Python 模块，被上面两个节点 import，必须一起装到同目录，
# 否则 `ros2 run` 起来的节点 import 不到它们。
install(PROGRAMS
  src/acg720_driver.py
  src/acg720_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  src/acg720_protocol.py
  src/drive_kinematics.py
  DESTINATION lib/${PROJECT_NAME}
)

# 注意：--symlink-install 的符号链接是**按文件**建的，不是按目录。
# 所以每次在下面这些目录里**新增**文件（新的 launch、新的 rviz 配置、
# 新的地图、新的脚本 …），都必须重新 colcon build --symlink-install 一次，
# install/ 里才会出现对应的链接。改已有文件的内容不需要重新 build。

ament_package()
