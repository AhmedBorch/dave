"""ROS 2 node: subscribe to submap PointCloud2, publish vehicle pose."""
import math
import os
import threading
from collections import deque
from typing import Optional

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

from .conversions import (
    pointcloud2_to_open3d,
    open3d_to_pointcloud2,
    matrix_to_quaternion,
    quaternion_to_matrix,
    pose_to_matrix,
    pose_with_covariance,
)
from .pipeline import PipelineParams, run
from .ransac import RansacParams
from .registration import GicpParams
from .uncertainty import UncertaintyParams


class MapMatchingNode(Node):
    def __init__(self) -> None:
        super().__init__('map_matching')

        self._declare_params()
        self._load_full_map()
        self._params = self._build_pipeline_params()
        self._T_world_struct = self._build_structure_pose()
        self._T_lidar_baselink = self._build_lidar_transform()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._busy = threading.Lock()
        self._world_frame = self.get_parameter('world_frame').get_parameter_value().string_value

        buf_size = self.get_parameter('vote_buffer_size').get_parameter_value().integer_value
        self._vote_cluster_radius = self.get_parameter('vote_cluster_radius').get_parameter_value().double_value
        # Each entry: (translation np.ndarray[3], quaternion np.ndarray[4 xyzw])
        self._pose_buffer: deque = deque(maxlen=buf_size)

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('output_pose_topic').get_parameter_value().string_value,
            10,
        )
        self._voted_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('average_pose_topic').get_parameter_value().string_value,
            10,
        )
        self._voted_cloud_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter('average_cloud_topic').get_parameter_value().string_value,
            10,
        )
        self._sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('input_cloud_topic').get_parameter_value().string_value,
            self._on_cloud,
            sensor_qos,
        )

        self.get_logger().info(
            f"map_matching ready: target={len(self._full_map.points)} pts, "
            f"world_frame='{self._world_frame}'"
        )

    def _declare_params(self) -> None:
        self.declare_parameter('full_map_path', '')
        self.declare_parameter('input_cloud_topic', '/cloud_in')
        self.declare_parameter('output_pose_topic', '~/vehicle_pose')
        self.declare_parameter('average_pose_topic', '~/average_pose')
        self.declare_parameter('average_cloud_topic', '~/average_cloud')
        self.declare_parameter('world_frame', 'map')

        # Structure pose in world frame
        self.declare_parameter('structure_position', [8.0, 0.0, 0.0])
        self.declare_parameter('structure_orientation_xyzw', [0.0, 0.0, 0.0, 1.0])

        # RANSAC
        self.declare_parameter('ransac.voxel_size', 0.03)
        self.declare_parameter('ransac.max_iterations', 100_000)

        # GICP
        self.declare_parameter('gicp.max_correspondence_distance', 0.5)
        self.declare_parameter('gicp.max_iterations', 50)
        self.declare_parameter('gicp.voxel_size', 0.03)

        # Uncertainty
        self.declare_parameter('uncertainty.base_translation_var', 1e-3)
        self.declare_parameter('uncertainty.base_rotation_var', 1e-4)

        # Fault check
        self.declare_parameter('roll_pitch_limit_deg', 10.0)
        self.declare_parameter('max_ransac_retries', 3)

        # Lidar mount in base_link frame (from SDF <pose relative_to="base_link">)
        self.declare_parameter('lidar_position', [0.25, 0.0, 0.05])
        self.declare_parameter('lidar_orientation_xyzw', [0.0, 0.0, 0.0, 1.0])

        # Voting / mode filter
        self.declare_parameter('vote_buffer_size', 30)
        self.declare_parameter('vote_cluster_radius', 0.3)

    def _load_full_map(self) -> None:
        path = self.get_parameter('full_map_path').get_parameter_value().string_value
        if not path or not os.path.isfile(path):
            raise FileNotFoundError(f"full_map_path is not a valid PCD file: '{path}'")
        cloud = o3d.io.read_point_cloud(path)
        if len(cloud.points) == 0:
            raise RuntimeError(f"Loaded full map is empty: '{path}'")
        self._full_map = cloud

    def _build_pipeline_params(self) -> PipelineParams:
        gp = self.get_parameter
        return PipelineParams(
            ransac=RansacParams(
                voxel_size=gp('ransac.voxel_size').get_parameter_value().double_value,
                max_iterations=gp('ransac.max_iterations').get_parameter_value().integer_value,
            ),
            gicp=GicpParams(
                max_correspondence_distance=gp('gicp.max_correspondence_distance').get_parameter_value().double_value,
                max_iterations=gp('gicp.max_iterations').get_parameter_value().integer_value,
                voxel_size=gp('gicp.voxel_size').get_parameter_value().double_value,
            ),
            uncertainty=UncertaintyParams(
                base_translation_var=gp('uncertainty.base_translation_var').get_parameter_value().double_value,
                base_rotation_var=gp('uncertainty.base_rotation_var').get_parameter_value().double_value,
            ),
            roll_pitch_limit_deg=gp('roll_pitch_limit_deg').get_parameter_value().double_value,
            max_ransac_retries=gp('max_ransac_retries').get_parameter_value().integer_value,
        )

    def _build_structure_pose(self) -> np.ndarray:
        xyz = list(self.get_parameter('structure_position').get_parameter_value().double_array_value)
        quat = list(self.get_parameter('structure_orientation_xyzw').get_parameter_value().double_array_value)
        return pose_to_matrix(xyz, quat)

    def _build_lidar_transform(self) -> np.ndarray:
        """Return T_lidar_baselink = inv(T_baselink_lidar) built from SDF mount params."""
        xyz = list(self.get_parameter('lidar_position').get_parameter_value().double_array_value)
        quat = list(self.get_parameter('lidar_orientation_xyzw').get_parameter_value().double_array_value)
        return np.linalg.inv(pose_to_matrix(xyz, quat))

    def _on_cloud(self, msg: PointCloud2) -> None:
        if not self._busy.acquire(blocking=False):
            self.get_logger().debug('Busy, dropping submap.')
            return
        try:
            self._process(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Pipeline failed: {exc}')
        finally:
            self._busy.release()

    def _vote_pose(self, pose_world: np.ndarray) -> np.ndarray:
        """Add pose to buffer, find the densest cluster, return its centroid.

        Each pose in the buffer casts a vote for every other pose within
        vote_cluster_radius. The pose with the most votes (densest neighbourhood)
        is the mode; we return the mean of all poses in that cluster.
        """
        t = pose_world[:3, 3].copy()
        q = matrix_to_quaternion(pose_world[:3, :3])
        self._pose_buffer.append((t, q))

        n = len(self._pose_buffer)
        if n == 1:
            return pose_world

        translations = np.array([p[0] for p in self._pose_buffer])  # (n, 3)
        quaternions  = np.array([p[1] for p in self._pose_buffer])  # (n, 4)

        # Count neighbours within cluster_radius for each pose.
        # Using broadcasting: diffs[i,j] = ||t_i - t_j||
        diffs  = translations[:, None, :] - translations[None, :, :]  # (n, n, 3)
        dists  = np.linalg.norm(diffs, axis=-1)                        # (n, n)
        counts = (dists <= self._vote_cluster_radius).sum(axis=1)      # (n,)

        best_idx = int(np.argmax(counts))
        mask = dists[best_idx] <= self._vote_cluster_radius
        cluster_size = int(mask.sum())

        # Mean translation of the winning cluster.
        voted_t = translations[mask].mean(axis=0)

        # Mean quaternion with hemisphere alignment relative to the cluster seed.
        ref_q = quaternions[best_idx]
        cluster_qs = quaternions[mask].copy()
        for i in range(cluster_size):
            if np.dot(cluster_qs[i], ref_q) < 0.0:
                cluster_qs[i] = -cluster_qs[i]
        voted_q = cluster_qs.mean(axis=0)
        voted_q /= np.linalg.norm(voted_q)

        voted_pose = np.eye(4)
        voted_pose[:3, :3] = quaternion_to_matrix(voted_q)
        voted_pose[:3, 3]  = voted_t
        return voted_pose

    def _process(self, msg: PointCloud2) -> None:
        submap = pointcloud2_to_open3d(msg)
        if len(submap.points) < 10:
            self.get_logger().warn('Submap has <10 points, skipping.')
            return

        result = run(submap, self._full_map, self._T_world_struct, self._params)

        if not result.is_valid:
            self.get_logger().warn(
                f'Registration rejected: roll={result.roll_deg:.1f}° pitch={result.pitch_deg:.1f}° '
                f'(limit ±{self._params.roll_pitch_limit_deg:.0f}°) after {result.attempts} attempt(s) — not publishing.'
            )
            return

        # Convert lidar-frame result to base_link frame: T_world_baselink = T_world_lidar @ T_lidar_baselink
        pose_baselink = result.pose_world @ self._T_lidar_baselink

        # Raw pose on the main topic.
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._world_frame
        out.pose = pose_with_covariance(pose_baselink, result.covariance)
        self._pub.publish(out)

        # Voted (modal-cluster) pose on the secondary topic.
        voted_pose = self._vote_pose(pose_baselink)
        voted_out = PoseWithCovarianceStamped()
        voted_out.header.stamp = msg.header.stamp
        voted_out.header.frame_id = self._world_frame
        voted_out.pose = pose_with_covariance(voted_pose, result.covariance)
        self._voted_pub.publish(voted_out)

        voted_cloud = o3d.geometry.PointCloud(submap).transform(voted_pose)
        cloud_out = open3d_to_pointcloud2(voted_cloud, voted_out.header)
        self._voted_cloud_pub.publish(cloud_out)

        R = voted_pose[:3, :3]
        voted_pitch = math.degrees(math.asin(float(np.clip(-R[2, 0], -1.0, 1.0))))
        voted_roll  = math.degrees(math.atan2(float(R[2, 1]), float(R[2, 2])))
        voted_yaw   = math.degrees(math.atan2(float(R[1, 0]), float(R[0, 0])))

        buf_n = len(self._pose_buffer)
        self.get_logger().info(
            f'GICP fitness={result.gicp.fitness:.3f} rmse={result.gicp.inlier_rmse:.3f}m  '
            f'roll={result.roll_deg:.1f}° pitch={result.pitch_deg:.1f}° yaw={result.yaw_deg:.1f}° '
            f'attempts={result.attempts} | '
            f'x={pose_baselink[0,3]:.3f} y={pose_baselink[1,3]:.3f} z={pose_baselink[2,3]:.3f} '
            f'voted(n={buf_n}): '
            f'x={voted_pose[0,3]:.3f} y={voted_pose[1,3]:.3f} z={voted_pose[2,3]:.3f} '
            f'roll={voted_roll:.1f}° pitch={voted_pitch:.1f}° yaw={voted_yaw:.1f}°'
        )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = MapMatchingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
