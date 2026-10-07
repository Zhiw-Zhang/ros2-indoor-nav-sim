"""真机 bringup：ACG720 底盘 + TF + Nav2（不含 Gazebo、不含仿真时钟）。

    ros2 launch my_robot_description robot.launch.py port:=/dev/ttyUSB0

它把三段串起来：

    acg720_driver          /cmd_vel → 串口；遥测 → /odom + odom→base_link
    robot_state_publisher  URDF → base_link→laser_frame 等固定 TF
    nav.launch.py sim:=false   map_server + AMCL + Nav2 + RViz
                                 （use_sim_time 自动为 false）

★ 与仿真 launch 的关键差别
--------------------------
**真机上没有 /clock。** 所以 use_sim_time 必须是 false。nav.launch.py 现在会
跟随 sim:=false 自动设成 false；这里再显式传一次，避免将来有人误改。

★ 默认是"能验证但不冒险"的配置
------------------------------
`use_laser` 默认 **false**：2D 雷达还没到货，所以默认不起雷达驱动，
Nav2 的代价地图会因为没有 /scan 而无法激活定位——这是**预期行为**。

    第一轮（台架/架空车轮）：只验底盘
        ros2 launch my_robot_description robot.launch.py nav:=false
        # 看遥控能不能动、/odom 方向对不对、急停管不管用

    第二轮（雷达到货后）：全套
        ros2 launch my_robot_description robot.launch.py use_laser:=true

常用参数
--------
    port:=/dev/ttyUSB0      串口设备
    nav:=false              只起底盘 + TF，不起 Nav2（第一轮就用这个）
    rviz:=true              开 RViz（默认关，省内存）
    use_laser:=true         起雷达驱动（需要先装好对应驱动包）
    lidar_pkg / lidar_launch  雷达驱动的包名与 launch 文件名
    wheel_separation_scale   ★ 标定值，默认 1.0，标完填这里

⚠ 说明书
--------
参数**怎么量、标完填哪**见 docs/real_robot_params.md 与 docs/acg720_protocol.md。
分阶段怎么做、每阶段的通过判据见 docs/real_robot_plan.md 第 6 节。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('my_robot_description')
    cfg = context.launch_configurations

    port = cfg.get('port', '/dev/ttyUSB0')
    baudrate = cfg.get('baudrate', '115200')
    nav_on = str(cfg.get('nav', 'true')).lower() in ('true', '1')
    rviz_on = cfg.get('rviz', 'false')
    use_laser = str(cfg.get('use_laser', 'false')).lower() in ('true', '1')
    scale = cfg.get('wheel_separation_scale', '1.0')
    counts_per_rev = cfg.get('counts_per_rev', '1320.0')
    wheel_radius = cfg.get('wheel_radius_m', '0.0325')
    wheel_separation = cfg.get('wheel_separation_m', '0.200')
    wheelbase = cfg.get('wheelbase_m', '0.185')
    max_linear_vel = cfg.get('max_linear_vel', '0.20')
    max_angular_vel = cfg.get('max_angular_vel', '1.0')
    max_wheel_rpm = cfg.get('max_wheel_rpm', '40.0')
    cmd_vel_timeout = cfg.get('cmd_vel_timeout', '0.5')
    map_yaml = cfg.get('map', '')
    log_level = cfg.get('log_level', 'info')

    urdf_path = os.path.join(pkg_share, 'urdf', 'acg720_real.urdf.xacro')
    if not os.path.isfile(urdf_path):
        raise RuntimeError(f'找不到真车 URDF: {urdf_path}')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str)

    actions = []

    # ---- 1. 底盘驱动 ----
    actions.append(Node(
        package='my_robot_description',
        executable='acg720_driver.py',
        name='acg720_driver',
        output='screen',
        parameters=[{
            'port': port,
            'baudrate': int(baudrate),
            'wheel_radius_m': float(wheel_radius),
            'wheel_separation_m': float(wheel_separation),
            'wheelbase_m': float(wheelbase),
            'counts_per_rev': float(counts_per_rev),
            # ★ 标定值。默认 1.0 = 不做修正，标完再填。
            #   不要抄仿真的 1.36（那是 ODE 物理的产物）。
            'wheel_separation_scale': float(scale),
            'max_linear_vel': float(max_linear_vel),
            'max_angular_vel': float(max_angular_vel),
            'max_wheel_rpm': float(max_wheel_rpm),
            'min_wheel_rpm': -float(max_wheel_rpm),
            'cmd_vel_timeout': float(cmd_vel_timeout),
            # 真机时钟：不是仿真时间
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    ))

    # ---- 2. 机器人状态发布：base_link→laser_frame 等固定 TF ----
    # ★ 这里**只**发固定关节。odom→base_link 由驱动节点发，
    #   两边都发会出现 TF_REPEATED_DATA、RViz 里车抖。
    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    ))

    # ---- 3. 雷达（可选；2D 雷达未到货前默认不起）----
    if use_laser:
        lidar_pkg = cfg.get('lidar_pkg', '')
        lidar_launch = cfg.get('lidar_launch', '')
        if not lidar_pkg or not lidar_launch:
            raise RuntimeError(
                'use_laser:=true 时必须同时给 lidar_pkg 和 lidar_launch，'
                '例如 lidar_pkg:=ldlidar_ros2 lidar_launch:=ld19.launch.py'
                '——本工程不绑定具体雷达型号，避免装错驱动还静默不出 /scan')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(lidar_pkg), 'launch', lidar_launch)),
            launch_arguments={
                # ★ frame 名必须是 laser_frame：URDF、nav2_params.yaml、
                #   slam_toolbox.yaml 三处都引用这个名字
                'frame_id': 'laser_frame',
                'use_sim_time': 'false',
            }.items(),
        ))

    else:
        # 明确地把"没起雷达"讲出来。不说的话，第一轮验证时看到
        # Nav2 的代价地图一直不激活、AMCL 没 /scan，很容易怀疑是驱动坏了。
        actions.append(LogInfo(msg=(
            '\n[robot.launch] ⚠ use_laser:=false（默认）——没有启动任何雷达驱动。\n'
            '  · 这是预期的：2D 雷达未到货。\n'
            '  · 因此不会有 /scan，Nav2 的 costmap/AMCL 无法正常激活。\n'
            '  · 第一轮只验底盘：ros2 launch my_robot_description robot.launch.py'
            ' nav:=false\n'
            '    遥控 /cmd_vel 看轮子转向、ros2 topic echo /odom 看方向与尺度。\n'
            '  · 雷达到货后用：use_laser:=true lidar_pkg:=<包名>'
            ' lidar_launch:=<launch 文件名>\n')))

    # ---- 4. Nav2（sim:=false，所以 use_sim_time 也是 false）----
    if nav_on:
        nav_args = {
            'sim': 'false',
            # 显式再传一次：真机没有 /clock
            'use_sim_time': 'false',
            'rviz': 'true' if str(rviz_on).lower() in ('true', '1') else 'false',
            'log_level': log_level,
        }
        if map_yaml:
            nav_args['map'] = map_yaml
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_share, 'launch', 'nav.launch.py')),
            launch_arguments=nav_args.items(),
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyUSB0',
            description='底盘串口设备（USB 转串口一般是 /dev/ttyUSB0）'),
        DeclareLaunchArgument(
            'baudrate', default_value='115200', description='串口波特率'),
        DeclareLaunchArgument(
            'nav', default_value='true',
            description='是否同时启动 Nav2。第一轮架空验证底盘时设 false'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='是否启动 RViz（默认关，省内存）'),
        DeclareLaunchArgument(
            'use_laser', default_value='false',
            description='是否启动雷达驱动。2D 雷达未到货前保持 false；'
                        '为 true 时必须同时给 lidar_pkg 和 lidar_launch'),
        DeclareLaunchArgument(
            'lidar_pkg', default_value='',
            description='雷达驱动的 ROS 包名（本工程不绑定型号）'),
        DeclareLaunchArgument(
            'lidar_launch', default_value='',
            description='雷达驱动的 launch 文件名'),
        # ---- 几何与标定（默认值全部来自交接包，均为【待确认】）----
        DeclareLaunchArgument(
            'wheel_separation_scale', default_value='1.0',
            description='★ skid-steer 有效轮距系数。默认 1.0=不修正；'
                        '按 docs/real_robot_params.md 第二节标定后填入。'
                        '不要抄仿真里的 1.36'),
        DeclareLaunchArgument(
            'counts_per_rev', default_value='1320.0',
            description='编码器每输出轴圈计数【待确认】'),
        DeclareLaunchArgument(
            'wheel_radius_m', default_value='0.0325', description='轮半径 m【待确认】'),
        DeclareLaunchArgument(
            'wheel_separation_m', default_value='0.200', description='轮距 m【待确认】'),
        DeclareLaunchArgument(
            'wheelbase_m', default_value='0.185', description='轴距 m【待确认】'),
        DeclareLaunchArgument(
            'max_linear_vel', default_value='0.20',
            description='最大线速度 m/s。比仿真的 0.5 保守，上车先从 0.15~0.2 起'),
        DeclareLaunchArgument(
            'max_angular_vel', default_value='1.0', description='最大角速度 rad/s'),
        DeclareLaunchArgument(
            'max_wheel_rpm', default_value='40.0',
            description='单轮最大 RPM。协议取值范围 8–40'),
        DeclareLaunchArgument(
            'cmd_vel_timeout', default_value='0.5',
            description='/cmd_vel 超时停车时间 s'),
        DeclareLaunchArgument(
            'map', default_value='', description='地图 yaml 绝对路径'),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='日志级别'),
        OpaqueFunction(function=_launch_setup),
    ])
