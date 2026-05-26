"""End-to-end pipeline: submap + full map -> vehicle pose in world frame.

Frames
------
- submap (source): point cloud expressed in the vehicle/sensor frame.
- full map (target): point cloud expressed in the structure's local frame.
- T_world_struct: pose of the structure in the world frame (input).

Registration yields T_struct_vehicle (the source-to-target transform).
Vehicle pose in world frame: T_world_vehicle = T_world_struct @ T_struct_vehicle.

Fault check
-----------
After registration, roll and pitch are extracted from pose_world (ZYX Euler).
If either exceeds roll_pitch_limit_deg the result is flagged is_valid=False and
RANSAC is rerun (it is stochastic — each call produces a different coarse guess).
Up to max_ransac_retries additional attempts are made.  If all fail, the attempt
with the smallest combined |roll|+|pitch| is returned with is_valid=False so the
caller can decide how to handle it.
"""
import math
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

    # Fault check: reject results whose recovered roll or pitch exceeds this.
    roll_pitch_limit_deg: float = 10.0

    # How many additional RANSAC attempts to make when a result is rejected.
    # Total attempts = 1 + max_ransac_retries.
    max_ransac_retries: int = 3


@dataclass
class MatchResult:
    pose_world: np.ndarray      # 4x4 vehicle pose in world frame
    covariance: np.ndarray      # 6x6 [x,y,z,roll,pitch,yaw]
    gicp: GicpResult            # raw registration output (source->struct)
    coarse_transform: np.ndarray  # 4x4 from the RANSAC step that produced this result
    roll_deg: float             # recovered roll  (deg, ZYX Euler)
    pitch_deg: float            # recovered pitch (deg, ZYX Euler)
    is_valid: bool              # False if roll/pitch exceeded roll_pitch_limit_deg
    attempts: int               # number of RANSAC runs used


def _roll_pitch_deg(T: np.ndarray):
    """Extract roll and pitch in degrees from a 4x4 pose matrix (ZYX Euler)."""
    R = T[:3, :3]
    # ZYX convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    #   R[2,0] = -sin(pitch)
    #   R[2,1] =  cos(pitch)*sin(roll)
    #   R[2,2] =  cos(pitch)*cos(roll)
    pitch = math.degrees(math.asin(float(np.clip(-R[2, 0], -1.0, 1.0))))
    roll  = math.degrees(math.atan2(float(R[2, 1]), float(R[2, 2])))
    return roll, pitch


def run(submap: o3d.geometry.PointCloud,
        full_map: o3d.geometry.PointCloud,
        T_world_struct: Optional[np.ndarray] = None,
        params: PipelineParams = PipelineParams()) -> MatchResult:

    if T_world_struct is None:
        T_world_struct = np.eye(4)

    limit = params.roll_pitch_limit_deg
    max_attempts = 1 + params.max_ransac_retries
    best: Optional[MatchResult] = None

    for attempt in range(1, max_attempts + 1):
        T_coarse    = align(submap, full_map, params.ransac)
        gicp_result = refine(submap, full_map, T_coarse, params.gicp)
        pose_world  = T_world_struct @ gicp_result.transformation
        cov         = estimate(gicp_result, params.uncertainty)

        roll, pitch = _roll_pitch_deg(pose_world)
        is_valid    = abs(roll) <= limit and abs(pitch) <= limit

        result = MatchResult(
            pose_world=pose_world,
            covariance=cov,
            gicp=gicp_result,
            coarse_transform=T_coarse,
            roll_deg=roll,
            pitch_deg=pitch,
            is_valid=is_valid,
            attempts=attempt,
        )

        if is_valid:
            return result

        # Keep the attempt with smallest combined angular error as fallback.
        if best is None or (abs(roll) + abs(pitch)) < (abs(best.roll_deg) + abs(best.pitch_deg)):
            best = result

    return best  # all attempts failed — caller checks is_valid
