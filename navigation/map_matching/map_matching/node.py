"""ROS 2 node: subscribe to submap PointCloud2, publish vehicle pose."""
import os
import threading
from typing import Optional

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

from .conversions import pointcloud2_to_open3d, pose_to_matrix, pose_with_covariance
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

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._busy = threading.Lock()
        self._world_frame = self.get_parameter('world_frame').get_parameter_value().string_value

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('output_pose_topic').get_parameter_value().string_value,
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
        )

    def _build_structure_pose(self) -> np.ndarray:
        xyz = list(self.get_parameter('structure_position').get_parameter_value().double_array_value)
        quat = list(self.get_parameter('structure_orientation_xyzw').get_parameter_value().double_array_value)
        return pose_to_matrix(xyz, quat)

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

    def _process(self, msg: PointCloud2) -> None:
        submap = pointcloud2_to_open3d(msg)
        if len(submap.points) < 10:
            self.get_logger().warn('Submap has <10 points, skipping.')
            return

        result = run(submap, self._full_map, self._T_world_struct, self._params)

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._world_frame
        out.pose = pose_with_covariance(result.pose_world, result.covariance)
        self._pub.publish(out)

        self.get_logger().info(
            f'GICP fitness={result.gicp.fitness:.3f} rmse={result.gicp.inlier_rmse:.3f}m'
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
