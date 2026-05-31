from scipy.spatial.transform import Rotation as R
import math

def get_yaw_from_quaternion(quat_or_tuple):
    if hasattr(quat_or_tuple, 'x'):
        x, y, z, w = quat_or_tuple.x, quat_or_tuple.y, quat_or_tuple.z, quat_or_tuple.w
    else:
        x, y, z, w = quat_or_tuple
    r = R.from_quat([x, y, z, w])
    euler = r.as_euler('xyz', degrees=False)
    return euler[2]  # Yaw


def normalize_angle(angle):
    """將角度正規化到 -pi 到 pi"""
    return math.atan2(math.sin(angle), math.cos(angle))