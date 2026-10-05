"""
纯几何工具 —— 不依赖 ROS，便于独立测试。

包含:
  * 四元数 / 欧拉角转换
  * 2D 光线投射（射线 vs 轴对齐矩形）
"""
import math

import numpy as np


def rpy_to_quat(roll, pitch, yaw):
    """欧拉角 -> 四元数 (x, y, z, w)"""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_to_yaw(qx, qy, qz, qw):
    """四元数 -> yaw（仅取偏航角）"""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def raycast(origin_x, origin_y, yaw, beam_angles,
            box_min, box_max, range_inf=np.inf):
    """
    向量化 2D 光线投射。

    参数:
      origin_x, origin_y : 射线起点（机器人基座在世界系中的位置）
      yaw                : 机器人朝向（弧度）
      beam_angles        : (N,) 每束光相对机器人前进方向的角度
      box_min, box_max   : (M,2) 障碍物矩形的左下/右上角

    返回:
      (N,) 距离数组，未命中为 +inf

    方法: 标准 slab 算法（射线 vs 轴对齐包围盒）。
    """
    beam_angles = np.asarray(beam_angles, dtype=np.float64)
    n_beams = beam_angles.shape[0]
    box_min = np.asarray(box_min, dtype=np.float64).reshape(-1, 2)
    box_max = np.asarray(box_max, dtype=np.float64).reshape(-1, 2)

    if box_min.shape[0] == 0:
        return np.full(n_beams, np.inf)

    world_angles = yaw + beam_angles
    dirs = np.stack([np.cos(world_angles), np.sin(world_angles)], axis=1)
    origin = np.array([origin_x, origin_y], dtype=np.float64)

    # 避免除零：把接近 0 的方向分量替换成一个极小值
    safe_dirs = np.where(np.abs(dirs) < 1e-12, 1e-12, dirs)

    best = np.full(n_beams, np.inf)

    for bi in range(box_min.shape[0]):
        bmin = box_min[bi]
        bmax = box_max[bi]

        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (bmin - origin) / safe_dirs
            t2 = (bmax - origin) / safe_dirs

        t_near = np.minimum(t1, t2).max(axis=1)
        t_far = np.maximum(t1, t2).min(axis=1)

        hit = (t_far >= np.maximum(t_near, 0.0)) & (t_far > 0.0)
        dist = np.where(hit, np.maximum(t_near, 0.0), range_inf)

        best = np.minimum(best, dist)

    return best
