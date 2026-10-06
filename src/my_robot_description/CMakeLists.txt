cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch worlds config maps docs
  DESTINATION share/${PROJECT_NAME}
)

# 工具脚本，装到 lib/${PROJECT_NAME} 下，用 `ros2 run` 调用：
#     ros2 run my_robot_description mapeval.py maps/rooms.yaml
#     ros2 run my_robot_description nav_goals.py -2.5,1.9 2.5,1.9
#     ros2 run my_robot_description xwd2png.py in.xwd out.png
# 用 install(PROGRAMS ...) 而不是 install(FILES ...)，前者会带上可执行位。
install(PROGRAMS
  scripts/mapeval.py
  scripts/nav_goals.py
  scripts/xwd2png.py
  DESTINATION lib/${PROJECT_NAME}
)

# 注意：--symlink-install 的符号链接是**按文件**建的，不是按目录。
# 所以每次在下面这些目录里**新增**文件（新的 launch、新的 rviz 配置、
# 新的地图、新的脚本 …），都必须重新 colcon build --symlink-install 一次，
# install/ 里才会出现对应的链接。改已有文件的内容不需要重新 build。

ament_package()
