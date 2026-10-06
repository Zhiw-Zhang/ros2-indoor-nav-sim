"""一条命令把整套导航跑起来：Gazebo + Nav2（AMCL 定位 + 规划 + 控制）+ RViz。

    ros2 launch my_robot_description nav.launch.py

默认流程（和实车一致）：
    1. 加载已建好的地图 maps/rooms.yaml（map_server）
    2. AMCL 用 /scan + /odom 在图上定位，发布 map -> odom
    3. 在 RViz 里用工具栏的 "2D Goal Pose"（或 Nav2 面板）点一个目标
    4. 车自己规划并走过去。终点误差的实测结果见 docs/nav2.md

常用变体
--------
    # Gazebo 已经在跑了，只起 Nav2 + RViz
    ros2 launch my_robot_description nav.launch.py sim:=false

    # 不开 RViz（省内存 / 跑无头验证）
    ros2 launch my_robot_description nav.launch.py rviz:=false

    # 边建图边导航：slam_toolbox 顶替 AMCL + map_server 提供地图和 map->odom
    # （此时 map:= 会被忽略；SLAM 参数固定用 config/slam_toolbox.yaml）
    ros2 launch my_robot_description nav.launch.py slam:=true

    # 换地图 / 换 world / 换参数文件
    ros2 launch my_robot_description nav.launch.py map:=/path/to/other.yaml
    ros2 launch my_robot_description nav.launch.py world:=empty.sdf
    ros2 launch my_robot_description nav.launch.py params_file:=/path/to/nav2_params.yaml

话题链路（Jazzy 的 nav2_bringup 已经 remap 好，这里只是记录）
------------------------------------------------------------
    controller_server --cmd_vel_nav--> velocity_smoother
        --cmd_vel_smoothed--> collision_monitor --/cmd_vel--> ros_gz_bridge
        --> gz DiffDrive

踩过的坑（都在这里规避掉了）
----------------------------
1. **不能用 launch_arguments 给 gazebo_sim.launch.py 传 rviz:=false。**
   IncludeLaunchDescription 的 launch_arguments 会被推进入**外层** launch
   上下文，于是本文件下面那个 RViz 节点的 IfCondition('rviz') 也变成 false，
   RViz 静默地不启动（日志里一行都没有）。必须用 GroupAction 的
   launch_configurations 把它限制在子作用域里。

2. **Nav2 要比仿真晚一点起。** AMCL 在 activate 时要用 set_initial_pose 给的
   初值去算 map->odom，那一刻如果 odom->base_link 这条 TF 还不存在就白设了。
   实测机器人是在 launch 后约 10 s 才 spawn 出来（gz -r 加载 world 很快，
   ros_gz_sim create 由 TimerAction(10) 触发），所以这里给 15 s。
   sim:=false 时不延迟（Gazebo 早就跑着了）。

3. **传给 nav2_bringup 的布尔参数必须是大写的 Python 字面量。**
   nav2_bringup 里有用 PythonExpression 拼裸布尔字面量的地方：
       bringup_launch.py:      PythonExpression([slam, ' and ', use_localization])
       navigation_launch.py:   PythonExpression(['not ', use_composition])
   等价于 eval("false and true")。Python 只认 False/True，所以传小写的
   slam:=false 会让整个 launch 直接崩：
       Caught exception in launch (see debug for traceback):
           name 'false' is not defined
   上游默认值恰好是大写的 'False'/'True'，所以照抄不会炸，自己传小写就炸。
   本文件用 _pybool() 统一转换后再往下传。

4. **slam:=true 时不能走 nav2_bringup 自带的 slam 分支。**
   bringup_launch.py 在 slam:=True 时会 include slam_launch.py，但只传
   params_file 一个参数；而 slam_launch.py 里有这么一段：

       has_slam_toolbox_params = HasNodeParams(params_file, 'slam_toolbox')
       ... launch_arguments={'slam_params_file': params_file}
           condition=IfCondition(has_slam_toolbox_params)

   也就是说，**只有当 params_file 里存在 `slam_toolbox:` 这一节时，它才会把
   参数传下去**。我们的 config/nav2_params.yaml 是纯 Nav2 参数，没有那一节，
   于是 slam_toolbox 会去读上游默认的 mapper_params_online_sync.yaml：

       base_frame: base_footprint                    ← 我们没有这个 link！
       check_min_dist_and_heading_precisely: false   ← 坑，见 config/slam_toolbox.yaml
       loop_search_maximum_distance: 3.0             ← 同上

   后果是 slam_toolbox 拿不到雷达 TF，/map 和 map->odom 一个都不发。
   表现极具迷惑性：Gazebo、Nav2、日志全部正常，**只有 RViz 里永远是一张
   空地图**，看起来像"SLAM + Nav2 这条路走不通"。

   本文件因此自己 include slam_toolbox 的 online_async_launch.py（参数只有
   config/slam_toolbox.yaml 一份真源），并给 nav2_bringup 传
   slam=False + use_localization=False，让它只起 navigation_launch.py。
   额外收益：用 online_async（后台线程处理扫描）而不是上游的 online_sync，
   车一边走一边建图时不容易丢帧；启动顺序也能自己控——**SLAM 必须先于
   Nav2**，否则 global_costmap 的 global_frame=map 在 activate 时找不到
   TF，lifecycle_manager 会直接激活失败。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _pybool(value):
    """把 'true'/'false'/'1'/'0' 统一成 Python 字面量 'True'/'False'。

    ★ 这是 nav2_bringup 的一个坑（本文件踩过）。

    nav2_bringup 里有好几处用 PythonExpression **拼裸布尔字面量**，例如
    bringup_launch.py:
        IfCondition(PythonExpression([slam, ' and ', use_localization]))
    localization_launch.py / navigation_launch.py:
        IfCondition(PythonExpression(['not ', use_composition]))

    它们最后是 `eval("false and true")`。Python 只认首字母大写的 False，
    所以小写会直接 NameError:
        Caught exception in launch (see debug for traceback):
            name 'false' is not defined
    上游的默认值恰好是 'False'/'True'（大写），所以照抄默认值不会炸，
    但你一旦自己传 slam:=true（小写，这是 launch 的常规写法）就炸。
    这里统一转成大写的 Python 字面量再往下传。
    """
    return 'True' if str(value).strip().lower() in ('true', '1') else 'False'


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('my_robot_description')
    nav2_share = get_package_share_directory('nav2_bringup')
    cfg = context.launch_configurations

    sim = cfg.get('sim', 'true')
    rviz = cfg.get('rviz', 'true')
    # ★ 走 _pybool：见上面的说明
    slam = _pybool(cfg.get('slam', 'false'))
    world = cfg.get('world', 'rooms.sdf')
    autostart = cfg.get('autostart', 'true')
    use_composition = _pybool(cfg.get('use_composition', 'true'))
    log_level = cfg.get('log_level', 'info')

    # 只做布尔判断用（slam 上面已经过 _pybool，sim 还没）
    sim_on = (_pybool(sim) == 'True')
    slam_on = (slam == 'True')

    map_yaml = cfg.get('map') or os.path.join(pkg_share, 'maps', 'rooms.yaml')
    params_file = cfg.get('params_file') or os.path.join(
        pkg_share, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    # slam:=true 时地图由 slam_toolbox 现场生成，map:= 会被忽略，
    # 所以那种情况下不检查地图文件是否存在。
    checks = [(params_file, 'Nav2 参数')]
    if not slam_on:
        checks.append((map_yaml, '地图'))
    else:
        checks.append((slam_params_file, 'SLAM 参数'))
    for path, what in checks:
        if not os.path.isfile(path):
            raise RuntimeError(f'找不到{what}文件: {path}')
    if not os.path.isfile(os.path.join(pkg_share, 'worlds', world)):
        raise RuntimeError(f'找不到 world 文件: {world}')

    actions = []

    # ---- 1. 仿真（不含它自己的 RViz；RViz 由本文件统一启动）----
    # 见文件头"坑 1"：rviz:=false 必须放在 GroupAction 的 launch_configurations
    # 里，绝不能走 launch_arguments。
    actions.append(GroupAction(
        condition=IfCondition(sim),
        launch_configurations={'rviz': 'false'},
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'gazebo_sim.launch.py')),
                launch_arguments={'world': world}.items(),
            ),
        ],
    ))

    # ---- 2. slam:=true 时，自己把 slam_toolbox 拉起来 ----
    # 为什么不走 nav2_bringup 自带的 slam 分支，见文件头"坑 4"。
    #
    # 用 online_async 而不是上游 slam_launch.py 里的 online_sync：
    # async 把扫描处理放在后台线程，车一边走一边建图时不容易丢帧。
    # 这和 slam.launch.py 用的是同一条路径、同一份参数文件。
    if slam_on:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': slam_params_file,
            }.items(),
        ))

    # ---- 3. Nav2 ----
    def nav2_include():
        """每次调用都新建一个 action 对象。

        不能把同一个 IncludeLaunchDescription 对象放进两个父 action 里
        —— launch 会把它访问两遍，两个 nav2_bringup 叠在一起跑。
        """
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'namespace': '',
                'use_namespace': 'false',
                # ★ 这里**永远**传 'False'，绝不把用户的 slam 参数直接透传给
                #   nav2_bringup —— 原因见文件头"坑 4"：它自带的 slam 分支在
                #   我们的 params_file 上会去读上游默认参数，base_frame 是
                #   base_footprint，结果 /map 和 map->odom 一个都不发。
                #   slam:=true 时 slam_toolbox 已由上面第 2 步自己拉起。
                'slam': 'False',
                # slam_on 时也不起 localization_launch.py（AMCL + map_server），
                # 让 nav2_bringup 只起 navigation_launch.py。
                # 注意 slam / use_localization / use_composition 都必须是
                # 大写的 Python 字面量，见 _pybool 的说明。
                'use_localization': 'False' if slam_on else 'True',
                'map': map_yaml,
                'use_sim_time': 'true',
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': use_composition,
                'use_respawn': 'false',
                'log_level': log_level,
            }.items(),
        )

    # Nav2 什么时候起：
    #   sim:=true           → 15 s。见"坑 2"：机器人大约 10 s 才 spawn 出来，
    #                          AMCL 激活时要用 set_initial_pose 算 map->odom，
    #                          那一刻 odom->base_link 必须已经存在。
    #   sim:=false + slam   → 8 s。Gazebo 早就跑着了，但要等 slam_toolbox
    #                          收到扫描并发出 map->odom，否则 global_costmap
    #                          的 global_frame=map 在 activate 时找不到 TF，
    #                          lifecycle_manager 会直接激活失败。
    #   sim:=false 且不 slam→ 0.5 s（等于立即，只是等一个 launch 事件循环）。
    nav2_delay = 15.0 if sim_on else (8.0 if slam_on else 0.5)
    actions.append(TimerAction(period=nav2_delay, actions=[nav2_include()]))

    # ---- 4. RViz（地图 + 代价地图 + 路径 + 粒子云 + Nav2 面板）----
    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav',
        arguments=['-d', os.path.join(pkg_share, 'config', 'nav_view.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen',
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='是否同时启动 Gazebo 仿真（Gazebo 已在跑就设 false）'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz2'),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='true=用 slam_toolbox 边建图边导航；'
                        'false=用 map_server+AMCL 在已有地图上定位（默认）'),
        DeclareLaunchArgument(
            'world', default_value='rooms.sdf',
            description='worlds/ 下的 world 文件名（仅 sim:=true 时有意义）'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='地图 yaml 的绝对路径，默认用包内 maps/rooms.yaml'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Nav2 参数文件，默认用包内 config/nav2_params.yaml'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='是否让 lifecycle_manager 自动 configure+activate'),
        DeclareLaunchArgument(
            'use_composition', default_value='true',
            description='用单个组件容器装所有 Nav2 节点（省内存，推荐）'),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='Nav2 节点日志级别'),
        OpaqueFunction(function=_launch_setup),
    ])
