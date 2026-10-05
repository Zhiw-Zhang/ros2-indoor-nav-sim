"""一条命令起全套：Gazebo 仿真 + slam_toolbox 建图 + RViz 看地图。

    ros2 launch my_robot_description slam.launch.py

如果 Gazebo 已经在另一个终端跑着，就加 sim:=false，只起 SLAM 和 RViz：

    ros2 launch my_robot_description slam.launch.py sim:=false

然后另开一个终端遥控，边走边看 RViz 里的地图长出来：

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

建好的地图保存成 pgm + yaml（供以后的 Nav2 / AMCL 用）：

    ros2 run nav2_map_server map_saver_cli -f ~/my_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')

    sim = LaunchConfiguration('sim')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')

    # 1. 仿真本体（rviz 交给下面这个带 SLAM 配置的 RViz）
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo_sim.launch.py')
        ),
        launch_arguments={'rviz': 'false', 'world': world}.items(),
        condition=IfCondition(sim),
    )

    # 2. slam_toolbox（online async：一边走一边建图，不阻塞）
    #    用官方 launch 是因为它顺手处理了 lifecycle 的 configure/activate 转换，
    #    自己写一份容易漏掉 EmitEvent 导致节点一直停在 unconfigured，
    #    表现是 /map 永远不出现。
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(pkg_share, 'config', 'slam_toolbox.yaml'),
        }.items(),
    )

    # 3. RViz：Fixed Frame 是 map，同时显示 /map 和 /scan
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', os.path.join(pkg_share, 'config', 'slam_view.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='是否连带启动 Gazebo 仿真（已经在跑就设 false）'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz2'),
        DeclareLaunchArgument(
            'world', default_value='rooms.sdf',
            description='worlds/ 下的 world 文件名'),
        sim_launch,
        slam_launch,
        rviz_node,
    ])
