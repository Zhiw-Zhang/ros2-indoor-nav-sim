import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _running_in_wsl():
    """检测是否运行在 WSL 下（读 /proc/version 或内核 release）。"""
    for path in ('/proc/version', '/proc/sys/kernel/osrelease'):
        try:
            with open(path) as f:
                if 'microsoft' in f.read().lower():
                    return True
        except OSError:
            pass
    return False


def _software_rendering_env():
    """WSL2 下必须强制软件渲染。

    硬件 GL 路径（Mesa 的 D3D12 后端）会让 Ogre2RenderEngine 初始化时
    直接段错误：
        [Wrn] [Ogre2RenderEngine.cc:551] Unable to open display: .
              Trying to run in headless mode.
        Segmentation fault
    改为 llvmpipe 后（其 D3D12 后端仍走 GPU）渲染线程正常初始化，
    gpu_lidar 读数与解析解一致（359/360 束误差 < 6cm）。
    实测额外开销仅约 3.5% 单核 / 370 MB 内存。

    另注：DISPLAY 必须保留（WSLg 提供 :0）。取消 DISPLAY 会让 OGRE2
    走 headless 分支，同样段错误。
    """
    return [
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
    ]


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')

    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # 1. 启动 Gazebo 并加载世界（GUI + server）
    #    曾经这里用过 -s（只开 server），因为怀疑 WSL2 上带 GUI 启动会让
    #    server 卡住、不注册 /world/<name>/create。后来证明那个卡住是
    #    empty.sdf 缺 user-commands-system 导致的，与 GUI 无关：补上插件后
    #    GUI 模式同样正常注册 create 服务，且 3D 视图渲染正常。
    #    因此恢复默认的 GUI + server 一起启动。
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 2. 发布机器人关节 TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # 3. 在 Gazebo 中生成机器人
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_robot',
                   '-z', '0.1'],
        output='screen'
    )

    # 4. ROS 2 <-> Gazebo 话题桥接
    #    /scan 由 URDF 里的 gpu_lidar 传感器发布，经 sensors-system 渲染产生。
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    actions = []
    # 软件渲染环境变量必须设在 gz_sim 启动之前
    if _running_in_wsl():
        actions.extend(_software_rendering_env())

    actions.extend([
        gz_sim,
        robot_state_publisher,
        # 10s 而非 3s：sensors 插件要等 OGRE2 渲染上下文就绪，world 加载
        # 明显慢于无传感器的世界，3s 时 create 常常拿不到 create 服务。
        TimerAction(period=10.0, actions=[spawn_robot]),
        bridge,
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch_setup)])
