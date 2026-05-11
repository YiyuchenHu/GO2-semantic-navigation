import math
from typing import Optional, Tuple

import numpy as np
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid


def distance_xy(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a[:2] - b[:2]))


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


def angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def occupancy_at_xy(grid: Optional[OccupancyGrid], x: float, y: float) -> Optional[int]:
    if grid is None:
        return None
    origin = grid.info.origin.position
    res = grid.info.resolution
    if res <= 0.0:
        return None
    mx = int((x - origin.x) / res)
    my = int((y - origin.y) / res)
    if mx < 0 or my < 0 or mx >= grid.info.width or my >= grid.info.height:
        return None
    idx = my * grid.info.width + mx
    return int(grid.data[idx])


def safe_cost(cost: Optional[int], threshold: int = 50) -> bool:
    if cost is None:
        return True
    if cost < 0:
        return True
    return cost < threshold


def entity_pose_xyz(entity) -> np.ndarray:
    return np.array(
        [entity.pose_map.position.x, entity.pose_map.position.y, entity.pose_map.position.z],
        dtype=np.float32,
    )


def odom_pose_xyz(odom) -> np.ndarray:
    return np.array(
        [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z],
        dtype=np.float32,
    )


def heading_to(src_xy: Tuple[float, float], dst_xy: Tuple[float, float]) -> float:
    return math.atan2(dst_xy[1] - src_xy[1], dst_xy[0] - src_xy[0])
