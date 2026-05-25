"""GICP refinement of the coarse RANSAC transform.

This is a placeholder using Open3D's generalized ICP. It will be replaced
with `small_gicp` (https://github.com/koide3/small_gicp) for a meaningful
speed-up; keep the `refine()` signature stable so callers don't change.
"""
from dataclasses import dataclass

import numpy as np
import open3d as o3d


@dataclass
class GicpParams:
    max_correspondence_distance: float = 0.5
    max_iterations: int = 50
    relative_fitness: float = 1e-6
    relative_rmse: float = 1e-6
    voxel_size: float = 0.03  # 0 → use full resolution


@dataclass
class GicpResult:
    transformation: np.ndarray   # 4x4 source->target
    fitness: float               # overlap ratio in [0, 1]
    inlier_rmse: float           # meters
    correspondence_set: np.ndarray  # Nx2 indices used for the final fit


def _maybe_downsample(cloud: o3d.geometry.PointCloud, voxel: float):
    return cloud.voxel_down_sample(voxel) if voxel > 0 else cloud


def refine(source: o3d.geometry.PointCloud,
           target: o3d.geometry.PointCloud,
           init_transform: np.ndarray,
           params: GicpParams = GicpParams()) -> GicpResult:
    """Refine an initial source->target transform via GICP."""
    src = _maybe_downsample(source, params.voxel_size)
    tgt = _maybe_downsample(target, params.voxel_size)

    if not src.has_normals():
        src.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=params.voxel_size * 4, max_nn=30))
    if not tgt.has_normals():
        tgt.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=params.voxel_size * 4, max_nn=30))

    result = o3d.pipelines.registration.registration_generalized_icp(
        src, tgt,
        max_correspondence_distance=params.max_correspondence_distance,
        init=init_transform,
        estimation_method=o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=params.relative_fitness,
            relative_rmse=params.relative_rmse,
            max_iteration=params.max_iterations,
        ),
    )

    return GicpResult(
        transformation=np.asarray(result.transformation),
        fitness=float(result.fitness),
        inlier_rmse=float(result.inlier_rmse),
        correspondence_set=np.asarray(result.correspondence_set),
    )
