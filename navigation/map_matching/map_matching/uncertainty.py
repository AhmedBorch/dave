"""Covariance estimators for the GICP pose output.

Two methods are available, selected by PipelineParams.use_brossard_cov:

  estimate()          — simple diagonal covariance scaled by GICP fitness/RMSE.
  estimate_brossard() — Censi (or Bonnabel) analytic covariance from Brossard
                        et al. "A New Approach to 3D ICP Covariance Estimation"
                        (RA-L 2020).  Calls the precompiled icp_with_cov binary
                        from /home/ahmed/map_matching/cov3D/3d-icp-cov/ via a
                        single-iteration PointMatcher run at the GICP solution,
                        then reads the 6×6 output and reorders it to match our
                        [x, y, z, roll, pitch, yaw] convention.

Covariance order throughout this package: [x, y, z, roll, pitch, yaw].
"""
import os
import subprocess
import tempfile
from dataclasses import dataclass

import numpy as np
import open3d as o3d

from .registration import GicpResult

# ---------------------------------------------------------------------------
# Paths to the Brossard library.
# Two environment variables let the same code work on the host and in Docker
# without any code changes:
#
#   BROSSARD_ROOT     — repo root (default: /home/ahmed/map_matching/cov3D/3d-icp-cov)
#   BROSSARD_BUILD    — build dir name inside libpointmatcher/
#                       host  → "build"        (compiled on host)
#                       Docker→ "build_docker" (compiled inside container)
# ---------------------------------------------------------------------------
_BROSSARD_ROOT = os.environ.get(
    'BROSSARD_ROOT',
    '/home/ahmed/map_matching/cov3D/3d-icp-cov',
)
_LPM_BUILD = os.path.join(
    _BROSSARD_ROOT, 'libpointmatcher',
    os.environ.get('BROSSARD_BUILD', 'build'),
)
_BINARY        = os.path.join(_LPM_BUILD, 'examples', 'icp_with_cov')
_DEFAULT_COV_CONFIG = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'brossard_cov_only.yaml'
)


# ---------------------------------------------------------------------------
# Existing (simple) estimator
# ---------------------------------------------------------------------------

@dataclass
class UncertaintyParams:
    base_translation_var: float = 1e-3   # m^2
    base_rotation_var: float = 1e-4      # rad^2
    rmse_translation_gain: float = 4.0
    rmse_rotation_gain: float = 1.0
    fitness_floor: float = 0.05          # avoid division by ~0
    max_translation_var: float = 1.0
    max_rotation_var: float = 0.25


def estimate(result: GicpResult,
             params: UncertaintyParams = UncertaintyParams()) -> np.ndarray:
    """Return a 6×6 diagonal covariance for the [x,y,z,roll,pitch,yaw] pose."""
    fitness = max(result.fitness, params.fitness_floor)
    rmse    = max(result.inlier_rmse, 0.0)

    trans_var = (params.base_translation_var
                 + params.rmse_translation_gain * rmse * rmse) / fitness
    rot_var   = (params.base_rotation_var
                 + params.rmse_rotation_gain * rmse * rmse) / fitness

    trans_var = min(trans_var, params.max_translation_var)
    rot_var   = min(rot_var,   params.max_rotation_var)

    cov = np.zeros((6, 6))
    cov[0, 0] = cov[1, 1] = cov[2, 2] = trans_var
    cov[3, 3] = cov[4, 4] = cov[5, 5] = rot_var
    return cov


# ---------------------------------------------------------------------------
# Brossard estimator
# ---------------------------------------------------------------------------

@dataclass
class BrossardParams:
    # Path to the PointMatcher config used for covariance-only evaluation.
    # Defaults to config/brossard_cov_only.yaml next to this file.
    config_path: str = ''
    # Sensor noise std dev used to scale the analytic covariance (metres).
    # Matches the lidar noise.stddev set in model.sdf.
    std_sensor: float = 0.01
    # True = use Bonnabel variant (sensor noise + calibration bias terms);
    # False = use Censi variant (sensor noise only, faster / more stable).
    use_bonnabel: bool = False
    # Voxel size for downsampling before saving to temp files.
    # Should match gicp.voxel_size so the point density is comparable.
    voxel_size: float = 0.05


def _str_vec(v: np.ndarray) -> str:
    """Format a 1-D array as '[a,b,c]' for the icp_with_cov --initTranslation arg."""
    return '[' + ','.join(str(x) for x in v) + ']'


def _str_mat(M: np.ndarray) -> str:
    """Format a 2-D array in row-major order as '[a,b,...,i]' for --initRotation."""
    return '[' + ','.join(str(x) for x in M.flatten()) + ']'


def _reorder_brossard_to_ours(cov6: np.ndarray) -> np.ndarray:
    """Permute a 6×6 covariance from Brossard's [rot|trans] to our [trans|rot].

    Brossard layout  (rows/cols): [roll, pitch, yaw, x, y, z]
    Our layout       (rows/cols): [x, y, z, roll, pitch, yaw]
    """
    out = np.zeros((6, 6))
    out[:3, :3] = cov6[3:, 3:]   # trans-trans
    out[3:, 3:] = cov6[:3, :3]   # rot-rot
    out[:3, 3:] = cov6[3:, :3]   # trans-rot cross
    out[3:, :3] = cov6[:3, 3:]   # rot-trans cross
    return out


def estimate_brossard(
    gicp_result: GicpResult,
    submap: o3d.geometry.PointCloud,
    full_map: o3d.geometry.PointCloud,
    params: BrossardParams = BrossardParams(),
) -> np.ndarray:
    """Compute Censi/Bonnabel ICP covariance via Brossard's icp_with_cov binary.

    The GICP-converged transform is passed as T_init.  The PointMatcher binary
    runs a single iteration (see brossard_cov_only.yaml) to evaluate the
    point-to-plane Jacobians at that solution, then outputs the analytic
    covariance without meaningfully changing the transform.

    Returns a 6×6 covariance ordered [x, y, z, roll, pitch, yaw].
    Falls back to None on any error (caller should use the simple estimator).
    """
    if not os.path.isfile(_BINARY):
        raise FileNotFoundError(
            f'icp_with_cov binary not found at {_BINARY}. '
            'Make sure the Brossard library is compiled.'
        )

    config = params.config_path or os.path.abspath(_DEFAULT_COV_CONFIG)
    if not os.path.isfile(config):
        raise FileNotFoundError(f'PointMatcher config not found: {config}')

    # Downsample to keep runtime reasonable (same density as GICP used)
    def _ds(cloud):
        return cloud.voxel_down_sample(params.voxel_size) if params.voxel_size > 0 else cloud

    ref_ds = _ds(full_map)
    src_ds = _ds(submap)

    # Write temp PCD files
    fd_ref,  ref_path  = tempfile.mkstemp(suffix='.pcd')
    fd_src,  src_path  = tempfile.mkstemp(suffix='.pcd')
    fd_pose, pose_path = tempfile.mkstemp(suffix='.txt')
    fd_cov,  cov_path  = tempfile.mkstemp(suffix='.txt')
    for fd in (fd_ref, fd_src, fd_pose, fd_cov):
        os.close(fd)

    try:
        o3d.io.write_point_cloud(ref_path, ref_ds)
        o3d.io.write_point_cloud(src_path, src_ds)

        T = gicp_result.transformation
        # Extend LD_LIBRARY_PATH so the binary finds libpointmatcher.so
        # regardless of whether it was compiled on the host or inside Docker.
        env = os.environ.copy()
        env['LD_LIBRARY_PATH'] = (
            _LPM_BUILD + ':' + env.get('LD_LIBRARY_PATH', '')
        )

        cmd = (
            f'cd {_LPM_BUILD} && '
            f'./examples/icp_with_cov'
            f' --config {config}'
            f' --output {pose_path}'
            f' --output_cov {cov_path}'
            f' --initTranslation {_str_vec(T[:3, 3])}'
            f' --initRotation {_str_mat(T[:3, :3])}'
            f' {ref_path} {src_path}'
        )
        subprocess.run(cmd, shell=True, capture_output=True, timeout=30, env=env)

        if not os.path.isfile(cov_path) or os.path.getsize(cov_path) == 0:
            raise RuntimeError('icp_with_cov produced no covariance output')

        cov_raw = np.genfromtxt(cov_path)   # shape (12, 6)
        if cov_raw.shape != (12, 6):
            raise RuntimeError(
                f'Unexpected covariance shape {cov_raw.shape}, expected (12, 6)'
            )

        # rows 0-5: Censi,  rows 6-11: Bonnabel
        raw = cov_raw[6:] if params.use_bonnabel else cov_raw[:6]
        cov_brossard = params.std_sensor ** 2 * raw

        return _reorder_brossard_to_ours(cov_brossard)

    finally:
        for p in (ref_path, src_path, pose_path, cov_path):
            try:
                os.unlink(p)
            except OSError:
                pass
