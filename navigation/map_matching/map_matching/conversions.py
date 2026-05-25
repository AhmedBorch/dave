"""Conversion helpers between ROS, NumPy, and Open3D types."""
import numpy as np
import open3d as o3d
from geometry_msgs.msg import Pose, PoseWithCovariance, Quaternion
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def pointcloud2_to_open3d(msg: PointCloud2) -> o3d.geometry.PointCloud:
    arr = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    if arr.size == 0:
        return o3d.geometry.PointCloud()
    points = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    subview = cloud.remove_non_finite_points()   # drop NaN / Inf entries
    # Optional: clip to the same range as the simulated sensor
    pts = np.asarray(subview.points)
    ranges = np.linalg.norm(pts, axis=1)
    mask = (ranges >= 0.2) & (ranges <= 15)
    subview.points = o3d.utility.Vector3dVector(pts[mask])
    return subview


def matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> [x, y, z, w] quaternion."""
    m = R
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    """[x, y, z, w] quaternion -> 3x3 rotation matrix."""
    n = np.linalg.norm(q)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_pose(T: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x = float(T[0, 3])
    pose.position.y = float(T[1, 3])
    pose.position.z = float(T[2, 3])
    q = matrix_to_quaternion(T[:3, :3])
    pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]),
                                  z=float(q[2]), w=float(q[3]))
    return pose


def pose_with_covariance(T: np.ndarray, cov_6x6: np.ndarray) -> PoseWithCovariance:
    msg = PoseWithCovariance()
    msg.pose = matrix_to_pose(T)
    msg.covariance = cov_6x6.flatten().tolist()
    return msg


def pose_to_matrix(xyz, quat_xyzw) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quaternion_to_matrix(np.asarray(quat_xyzw, dtype=float))
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T
