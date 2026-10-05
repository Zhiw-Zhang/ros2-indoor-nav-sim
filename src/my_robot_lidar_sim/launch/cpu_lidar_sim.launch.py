#!/usr/bin/env python3
"""
CPU 激光雷达仿真启动文件。

启动内容：
  1. Gazebo（仅物理引擎，不加载 sensors 系统 —— 避免 WSL2 渲染卡死）
  2. robot_state_publisher（关节 TF）
  3. 在 Gazebo 中生成机器人
  4. ros_gz_bridge（/cmd_vel -> GZ，GZ -> /odom、/tf、/clock；不含 /scan）
  5. cpu_lidar_node（CPU 光线投射，发布 /scan）

注意：/scan 由 cpu_lidar_node 提供，因此【不桥接】 Gazebo 的 /scan 话题。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_share = get_package_share_directory('my_robot_description')
    lidar_share = get_package_share_directory('my_robot_lidar_sim')

    urdf_file = os.path.join(robot_share, 'urdf', 'my_robot.urdf.xacro')
    world_file = os.path.join(robot_share, 'worlds', 'empty.sdf')
    lidar_config = os.path.join(lidar_share, 'config', 'lidar_config.yaml')

    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    use_cpu_lidar = LaunchConfiguration('use_cpu_lidar')
    use_gazebo = LaunchConfiguration('use_gazebo')

    # 1. Gazebo：仅物理引擎（world 中刻意未加载 sensors 系统）
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        condition=IfCondition(use_gazebo),
    )

    # 2. 机器人关节 TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 3. 在 Gazebo 中生成机器人
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_robot',
                   '-z', '0.1'],
        output='screen',
        condition=IfCondition(use_gazebo),
    )

    # 4. ROS <-> Gazebo 话题桥接
    #    刻意不含 /scan：激光雷达由 CPU 节点提供。
    #    也不桥接真实位姿：实测 gz.msgs.Pose_V -> geometry_msgs/PoseArray
    #    这个转换不会真正转发数据（ROS 侧 Publisher count 为 0），
    #    因此 cpu_lidar_node 直接读取 Gazebo 原生 /world/default/pose/info。
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_gazebo),
    )

    # 5. CPU 光线投射激光雷达（发布 /scan）
    cpu_lidar = Node(
        package='my_robot_lidar_sim',
        executable='cpu_lidar_node',
        name='cpu_lidar_node',
        parameters=[{'config_file': lidar_config, 'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_cpu_lidar),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gazebo', default_value='true',
            description='是否启动 Gazebo 物理仿真（提供 /odom、真实动力学）'),
        DeclareLaunchArgument(
            'use_cpu_lidar', default_value='true',
            description='是否启动 CPU 光线投射激光雷达（发布 /scan）'),

        gz_sim,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_robot]),
        bridge,
        # 稍等 Gazebo 起来后再启动雷达，避免首帧位姿未就绪
        TimerAction(period=4.0, actions=[cpu_lidar]),
    ])
