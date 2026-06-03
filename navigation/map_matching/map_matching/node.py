"""ROS 2 node: subscribe to submap PointCloud2, publish vehicle pose."""
import math
import os
import threading
from collections import deque
from typing import Optional

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
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
from .uncertainty import BrossardParams, UncertaintyParams


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

        # Quality-gate state
        self._min_points = self.get_parameter('min_points').get_parameter_value().integer_value
        self._warmup_count = self.get_parameter('warmup_measurements').get_parameter_value().integer_value
        self._consistency_max_t = self.get_parameter('consistency_max_translation_m').get_parameter_value().double_value
        self._consistency_max_r_deg = self.get_parameter('consistency_max_rotation_deg').get_parameter_value().double_value
        self._consistency_cov_scale = self.get_parameter('consistency_covariance_scale').get_parameter_value().double_value
        self._valid_count: int = 0
        self._last_pose: Optional[np.ndarray] = None
        self._last_voted_pose: Optional[np.ndarray] = None

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('output_pose_topic').get_parameter_value().string_value,
            10,
        )
        self._euler_pub = self.create_publisher(
            Vector3Stamped,
            self.get_parameter('output_euler_topic').get_parameter_value().string_value,
            10,
        )
        self._voted_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('average_pose_topic').get_parameter_value().string_value,
            10,
        )
        self._voted_euler_pub = self.create_publisher(
            Vector3Stamped,
            self.get_parameter('average_euler_topic').get_parameter_value().string_value,
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
        self.declare_parameter('output_euler_topic', '~/vehicle_euler')
        self.declare_parameter('average_pose_topic', '~/average_pose')
        self.declare_parameter('average_euler_topic', '~/average_euler')
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

        # Quality gates
        self.declare_parameter('min_points', 1000)
        self.declare_parameter('warmup_measurements', 20)
        self.declare_parameter('consistency_max_translation_m', 1.0)
        self.declare_parameter('consistency_max_rotation_deg', 10.0)
        self.declare_parameter('consistency_covariance_scale', 1000.0)

        # Covariance method
        self.declare_parameter('use_brossard_cov', False)
        self.declare_parameter('brossard.std_sensor', 0.01)
        self.declare_parameter('brossard.use_bonnabel', False)
        self.declare_parameter('brossard.voxel_size', 0.05)

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
            brossard=BrossardParams(
                std_sensor=gp('brossard.std_sensor').get_parameter_value().double_value,
                use_bonnabel=gp('brossard.use_bonnabel').get_parameter_value().bool_value,
                voxel_size=gp('brossard.voxel_size').get_parameter_value().double_value,
            ),
            roll_pitch_limit_deg=gp('roll_pitch_limit_deg').get_parameter_value().double_value,
            max_ransac_retries=gp('max_ransac_retries').get_parameter_value().integer_value,
            use_brossard_cov=gp('use_brossard_cov').get_parameter_value().bool_value,
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

    def _compute_vote(self, seed_pose: np.ndarray) -> np.ndarray:
        """Return the modal-cluster centroid of the current buffer.

        Does NOT modify the buffer. Falls back to seed_pose when the buffer
        has fewer than 2 entries.
        """
        n = len(self._pose_buffer)
        if n < 2:
            return seed_pose

        translations = np.array([p[0] for p in self._pose_buffer])  # (n, 3)
        quaternions  = np.array([p[1] for p in self._pose_buffer])  # (n, 4)

        diffs  = translations[:, None, :] - translations[None, :, :]  # (n, n, 3)
        dists  = np.linalg.norm(diffs, axis=-1)                        # (n, n)
        counts = (dists <= self._vote_cluster_radius).sum(axis=1)      # (n,)

        best_idx = int(np.argmax(counts))
        mask = dists[best_idx] <= self._vote_cluster_radius
        cluster_size = int(mask.sum())

        voted_t = translations[mask].mean(axis=0)

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

    def _add_and_vote(self, pose_world: np.ndarray) -> np.ndarray:
        """Add pose to buffer, then return the modal-cluster centroid."""
        t = pose_world[:3, 3].copy()
        q = matrix_to_quaternion(pose_world[:3, :3])
        self._pose_buffer.append((t, q))
        return self._compute_vote(pose_world)

    def _process(self, msg: PointCloud2) -> None:
        submap = pointcloud2_to_open3d(msg)
        if len(submap.points) < self._min_points:
            self.get_logger().warn(
                f'Submap has {len(submap.points)} points (<{self._min_points}), skipping.')
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

        # --- Consistency check vs previous valid measurement ---
        cov_scale = 1.0
        is_consistent = True
        if self._last_pose is not None:
            delta_t = float(np.linalg.norm(pose_baselink[:3, 3] - self._last_pose[:3, 3]))
            R_diff = self._last_pose[:3, :3].T @ pose_baselink[:3, :3]
            delta_r_deg = math.degrees(
                math.acos(float(np.clip((np.trace(R_diff) - 1.0) / 2.0, -1.0, 1.0)))
            )
            if delta_t > self._consistency_max_t or delta_r_deg > self._consistency_max_r_deg:
                is_consistent = False
                cov_scale = self._consistency_cov_scale
                self.get_logger().warn(
                    f'Inconsistent jump: Δt={delta_t:.2f}m Δr={delta_r_deg:.1f}° '
                    f'— publishing with covariance ×{cov_scale:.0f}'
                )
        self._last_pose = pose_baselink

        # --- Warmup: fill the vote buffer silently before publishing ---
        self._valid_count += 1
        if self._valid_count <= self._warmup_count:
            self._add_and_vote(pose_baselink)
            self.get_logger().info(
                f'Warming up: {self._valid_count}/{self._warmup_count} valid measurements'
            )
            return

        # --- Publish raw pose (scaled covariance when inconsistent) ---
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._world_frame
        out.pose = pose_with_covariance(pose_baselink, result.covariance * cov_scale)
        self._pub.publish(out)

        R_raw = pose_baselink[:3, :3]
        raw_euler = Vector3Stamped()
        raw_euler.header = out.header
        raw_euler.vector.x = math.atan2(float(R_raw[2, 1]), float(R_raw[2, 2]))
        raw_euler.vector.y = math.asin(float(np.clip(-R_raw[2, 0], -1.0, 1.0)))
        raw_euler.vector.z = math.atan2(float(R_raw[1, 0]), float(R_raw[0, 0]))
        self._euler_pub.publish(raw_euler)

        # --- Voted pose: only consistent measurements enter the buffer ---
        if is_consistent:
            voted_pose = self._add_and_vote(pose_baselink)
            self._last_voted_pose = voted_pose
        else:
            voted_pose = self._last_voted_pose if self._last_voted_pose is not None \
                else self._compute_vote(pose_baselink)

        voted_out = PoseWithCovarianceStamped()
        voted_out.header.stamp = msg.header.stamp
        voted_out.header.frame_id = self._world_frame
        voted_out.pose = pose_with_covariance(voted_pose, result.covariance)
        self._voted_pub.publish(voted_out)

        voted_cloud = o3d.geometry.PointCloud(submap).transform(voted_pose)
        cloud_out = open3d_to_pointcloud2(voted_cloud, voted_out.header)
        self._voted_cloud_pub.publish(cloud_out)

        R = voted_pose[:3, :3]
        voted_roll  = math.atan2(float(R[2, 1]), float(R[2, 2]))
        voted_pitch = math.asin(float(np.clip(-R[2, 0], -1.0, 1.0)))
        voted_yaw   = math.atan2(float(R[1, 0]), float(R[0, 0]))

        voted_euler = Vector3Stamped()
        voted_euler.header = voted_out.header
        voted_euler.vector.x = voted_roll
        voted_euler.vector.y = voted_pitch
        voted_euler.vector.z = voted_yaw
        self._voted_euler_pub.publish(voted_euler)

        buf_n = len(self._pose_buffer)
        self.get_logger().info(
            f'GICP fitness={result.gicp.fitness:.3f} rmse={result.gicp.inlier_rmse:.3f}m  '
            f'roll={result.roll_deg:.1f}° pitch={result.pitch_deg:.1f}° yaw={result.yaw_deg:.1f}° '
            f'attempts={result.attempts} consistent={is_consistent} | '
            f'x={pose_baselink[0,3]:.3f} y={pose_baselink[1,3]:.3f} z={pose_baselink[2,3]:.3f} '
            f'voted(n={buf_n}): '
            f'x={voted_pose[0,3]:.3f} y={voted_pose[1,3]:.3f} z={voted_pose[2,3]:.3f} '
            f'roll={math.degrees(voted_roll):.1f}° pitch={math.degrees(voted_pitch):.1f}° '
            f'yaw={math.degrees(voted_yaw):.1f}°'
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
