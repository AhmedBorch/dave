"""End-to-end pipeline: submap + full map -> vehicle pose in world frame.

Frames
------
- submap (source): point cloud expressed in the vehicle/sensor frame.
- full map (target): point cloud expressed in the structure's local frame.
- T_world_struct: pose of the structure in the world frame (input).

Registration yields T_struct_vehicle (the source-to-target transform).
Vehicle pose in world frame: T_world_vehicle = T_world_struct @ T_struct_vehicle.
"""
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import open3d as o3d

from .ransac import RansacParams, align
from .registration import GicpParams, GicpResult, refine
from .uncertainty import UncertaintyParams, estimate


@dataclass
class PipelineParams:
    ransac: RansacParams = field(default_factory=RansacParams)
    gicp: GicpParams = field(default_factory=GicpParams)
    uncertainty: UncertaintyParams = field(default_factory=UncertaintyParams)


@dataclass
class MatchResult:
    pose_world: np.ndarray         # 4x4 vehicle pose in world frame
    covariance: np.ndarray         # 6x6 [x,y,z,roll,pitch,yaw]
    gicp: GicpResult               # raw registration output (source->struct)
    coarse_transform: np.ndarray   # 4x4 from RANSAC step


def run(submap: o3d.geometry.PointCloud,
        full_map: o3d.geometry.PointCloud,
        T_world_struct: Optional[np.ndarray] = None,
        params: PipelineParams = PipelineParams()) -> MatchResult:
    if T_world_struct is None:
        T_world_struct = np.eye(4)

    T_coarse = align(submap, full_map, params.ransac)
    gicp_result = refine(submap, full_map, T_coarse, params.gicp)
    cov = estimate(gicp_result, params.uncertainty)

    pose_world = T_world_struct @ gicp_result.transformation

    return MatchResult(
        pose_world=pose_world,
        covariance=cov,
        gicp=gicp_result,
        coarse_transform=T_coarse,
    )
