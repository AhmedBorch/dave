"""Global feature-based alignment of submap to full map using RANSAC + FPFH.

Returns a 4x4 transform that maps points from the submap (source) frame
into the full-map (target) frame.
"""
from dataclasses import dataclass

import numpy as np
import open3d as o3d


@dataclass
class RansacParams:
    voxel_size: float = 0.03
    normal_radius_factor: float = 2.0
    feature_radius_factor: float = 5.0
    # distance_threshold_factor: float = 1.5
    distance_threshold_factor: float = 10.0
    max_iterations: int = 100_000
    confidence: float = 0.999


def _preprocess(cloud: o3d.geometry.PointCloud, voxel: float,
                normal_radius: float, feature_radius: float):
    down = cloud.voxel_down_sample(voxel)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down, o3d.geometry.KDTreeSearchParamHybrid(radius=feature_radius, max_nn=100))
    return down, fpfh


def align(source: o3d.geometry.PointCloud,
          target: o3d.geometry.PointCloud,
          params: RansacParams = RansacParams()) -> np.ndarray:
    """Run RANSAC global registration. Returns 4x4 source->target transform."""
    normal_radius = params.voxel_size * params.normal_radius_factor
    feature_radius = params.voxel_size * params.feature_radius_factor
    dist_thresh = params.voxel_size * params.distance_threshold_factor

    src_down, src_fpfh = _preprocess(source, params.voxel_size,
                                     normal_radius, feature_radius)
    tgt_down, tgt_fpfh = _preprocess(target, params.voxel_size,
                                     normal_radius, feature_radius)

    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_down, tgt_down, src_fpfh, tgt_fpfh,
        mutual_filter=True,
        max_correspondence_distance=dist_thresh,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist_thresh),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            params.max_iterations, params.confidence),
    )
    return np.asarray(result.transformation)
