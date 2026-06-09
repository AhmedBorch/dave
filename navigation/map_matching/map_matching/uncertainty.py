"""Covariance estimators for the GICP pose output.

Two methods are available, selected by PipelineParams.use_brossard_cov:

  estimate()          — simple diagonal covariance scaled by GICP fitness/RMSE.
  estimate_brossard() — Brossard et al. "A New Approach to 3D ICP Covariance
                        Estimation" (RA-L 2020).  Two modes via params.use_ut:

      use_ut = False  (analytic only):
          cov = σ_sensor² · (Censi | Bonnabel)
        from the precompiled icp_with_cov binary — a single linearisation step
        at the GICP solution.  Fast, but optimistic: it ignores the
        wrong-convergence / initialisation uncertainty.

      use_ut = True   (full method):
          cov = σ_sensor² · (Censi | Bonnabel)        [analytic term]
              + cov_ut(Q_ini)                          [unscented transform]
        The UT term propagates an initialisation prior Q_ini through GICP by
        re-running registration from 2n+1 = 13 sigma-point perturbations of the
        converged transform and accumulating the spread of the results.  This is
        the complete paper method but costs ~13 extra GICP runs per estimate.

The binary emits the covariance ordered [roll, pitch, yaw, x, y, z]; the UT
term is computed in the same [rot|trans] order, and the total is reordered to
this package's convention [x, y, z, roll, pitch, yaw] before returning.
"""
import os
import subprocess
import tempfile
from dataclasses import dataclass

import numpy as np
import open3d as o3d

from .registration import GicpParams, GicpResult, refine

# ---------------------------------------------------------------------------
# Paths to the Brossard library.
# Two environment variables let the same code work on the host and in Docker
# without any code changes:
#
#   BROSSARD_ROOT     — repo root (default: /../map_matching/cov3D/3d-icp-cov)
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
    # Full Brossard method: add the unscented-transform (UT) term on top of the
    # analytic term.  False → analytic only (fast).  True → analytic + UT term
    # (the complete RA-L 2020 method; re-runs GICP ~13× per estimate).
    use_ut: bool = False
    # Initialisation-prior difficulty level for the UT term:
    #   'easy'   → ±2°  rotation, ±5 cm  translation
    #   'medium' → ±10° rotation, ±25 cm translation
    #   'hard'   → ±30° rotation, ±75 cm translation
    q_ini_level: str = 'medium'


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


# ---------------------------------------------------------------------------
# SE(3) exp/log helpers, tangent convention xi = [omega; rho] (rotation first),
# matching Brossard's repo so the UT term is consistent with the binary's
# [rot|trans] covariance.
# ---------------------------------------------------------------------------

def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def _so3_exp_jac(phi: np.ndarray):
    """Return (R, J) — SO(3) exp and the left Jacobian for the translation."""
    angle = float(np.linalg.norm(phi))
    if angle < 1e-8:
        K = _skew(phi)
        return np.eye(3) + K, np.eye(3) + 0.5 * K
    axis = phi / angle
    K = _skew(axis)
    s, c = np.sin(angle), np.cos(angle)
    R = c * np.eye(3) + (1 - c) * np.outer(axis, axis) + s * K
    J = (s / angle) * np.eye(3) + (1 - s / angle) * np.outer(axis, axis) \
        + ((1 - c) / angle) * K
    return R, J


def _se3_exp(xi: np.ndarray) -> np.ndarray:
    R, J = _so3_exp_jac(xi[:3])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = J @ xi[3:]
    return T


def _so3_log(R: np.ndarray) -> np.ndarray:
    cos_angle = np.clip(0.5 * np.trace(R) - 0.5, -1.0, 1.0)
    angle = np.arccos(cos_angle)
    if np.isclose(angle, 0.0):
        return 0.5 * np.array([R[2, 1] - R[1, 2],
                               R[0, 2] - R[2, 0],
                               R[1, 0] - R[0, 1]])
    return (angle / (2 * np.sin(angle))) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def _inv_left_jacobian(phi: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(phi))
    if np.isclose(angle, 0.0):
        return np.eye(3) - 0.5 * _skew(phi)
    axis = phi / angle
    half = 0.5 * angle
    cot = 1.0 / np.tan(half)
    return half * cot * np.eye(3) + (1 - half * cot) * np.outer(axis, axis) \
        - half * _skew(axis)


def _se3_log(T: np.ndarray) -> np.ndarray:
    phi = _so3_log(T[:3, :3])
    rho = _inv_left_jacobian(phi) @ T[:3, 3]
    return np.hstack([phi, rho])


def _normalize_se3(T: np.ndarray) -> np.ndarray:
    """Re-orthonormalise the rotation block (drift from exp/compose)."""
    U, _, Vt = np.linalg.svd(T[:3, :3])
    S = np.eye(3)
    S[2, 2] = np.linalg.det(U) * np.linalg.det(Vt)
    out = np.eye(4)
    out[:3, :3] = U @ S @ Vt
    out[:3, 3] = T[:3, 3]
    return out


# ---------------------------------------------------------------------------
# Initialisation prior Q_ini and unscented sigma points
# ---------------------------------------------------------------------------

# (rot_std_deg, trans_std_m) per difficulty level. Q_ini is built as a diagonal
# 6×6 tangent covariance diag([rot_var]*3 + [trans_var]*3) in [omega; rho] order.
Q_INI_LEVELS = {
    'easy':   (2.0,  0.05),   # ±2°  rotation, ±5 cm  translation
    'medium': (10.0, 0.25),   # ±10° rotation, ±25 cm translation
    'hard':   (30.0, 0.75),   # ±30° rotation, ±75 cm translation
}


def build_Q_ini(rot_std_deg: float, trans_std_m: float) -> np.ndarray:
    rot_var = np.radians(rot_std_deg) ** 2
    trans_var = trans_std_m ** 2
    return np.diag([rot_var] * 3 + [trans_var] * 3)


def _resolve_q_ini(level: str) -> np.ndarray:
    if level not in Q_INI_LEVELS:
        raise ValueError(
            f"Unknown q_ini_level '{level}', expected one of {list(Q_INI_LEVELS)}")
    return build_Q_ini(*Q_INI_LEVELS[level])


def _sigma_points(Q: np.ndarray, alpha: float = 1.0, beta: float = 2.0,
                  kappa: float = 0.0):
    """Scaled unscented sigma points (2n+1) and covariance weights Wc.

    With Brossard's defaults (alpha=1, beta=2, kappa=0): lambda=0.
    """
    n = Q.shape[0]
    lam = alpha ** 2 * (n + kappa) - n
    U = np.linalg.cholesky((lam + n) * Q)   # rows used as sigma offsets
    sigmas = np.zeros((2 * n + 1, n))
    sigmas[1:n + 1] = U
    sigmas[n + 1:] = -U
    c = 0.5 / (n + lam)
    Wc = np.full(2 * n + 1, c)
    Wc[0] = lam / (n + lam) + (1 - alpha ** 2 + beta)
    return sigmas, Wc


# ---------------------------------------------------------------------------
# Analytic term (binary) and UT term
# ---------------------------------------------------------------------------

def _run_brossard_binary(
    gicp_result: GicpResult,
    submap: o3d.geometry.PointCloud,
    full_map: o3d.geometry.PointCloud,
    params: BrossardParams,
) -> np.ndarray:
    """Call icp_with_cov at the GICP solution, return 6×6 cov in [rot|trans].

    The PointMatcher binary runs a single linearisation step (see
    brossard_cov_only.yaml) to evaluate the point-to-plane Jacobians at the
    converged transform, then emits the Censi (rows 0-5) and Bonnabel (rows
    6-11) covariances. The requested one is scaled by std_sensor².
    """
    if not os.path.isfile(_BINARY):
        raise FileNotFoundError(
            f'icp_with_cov binary not found at {_BINARY}. '
            'Make sure the Brossard library is compiled.'
        )

    config = params.config_path or os.path.abspath(_DEFAULT_COV_CONFIG)
    if not os.path.isfile(config):
        raise FileNotFoundError(f'PointMatcher config not found: {config}')

    def _ds(cloud):
        return cloud.voxel_down_sample(params.voxel_size) if params.voxel_size > 0 else cloud

    ref_ds = _ds(full_map)
    src_ds = _ds(submap)

    fd_ref,  ref_path  = tempfile.mkstemp(suffix='.pcd')
    fd_src,  src_path  = tempfile.mkstemp(suffix='.pcd')
    fd_pose, pose_path = tempfile.mkstemp(suffix='.txt')
    fd_cov,  cov_path  = tempfile.mkstemp(suffix='.txt')
    for fd in (fd_ref, fd_src, fd_pose, fd_cov):
        os.close(fd)

    try:
        o3d.io.write_point_cloud(ref_path, ref_ds, write_ascii=True)
        o3d.io.write_point_cloud(src_path, src_ds, write_ascii=True)

        T = gicp_result.transformation
        env = os.environ.copy()
        env['LD_LIBRARY_PATH'] = _LPM_BUILD + ':' + env.get('LD_LIBRARY_PATH', '')

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
                f'Unexpected covariance shape {cov_raw.shape}, expected (12, 6)')

        # rows 0-5: Censi,  rows 6-11: Bonnabel — already [rot|trans]
        raw = cov_raw[6:] if params.use_bonnabel else cov_raw[:6]
        return params.std_sensor ** 2 * raw

    finally:
        for p in (ref_path, src_path, pose_path, cov_path):
            try:
                os.unlink(p)
            except OSError:
                pass


def _compute_cov_ut(
    submap: o3d.geometry.PointCloud,
    full_map: o3d.geometry.PointCloud,
    center_T: np.ndarray,
    Q: np.ndarray,
    gicp_params: GicpParams,
) -> np.ndarray:
    """Unscented-transform term: propagate the init prior Q through GICP.

    Generates 2n+1 sigma-point perturbations of the converged transform
    (T_init = exp(xi)·center_T), re-runs GICP from each, then accumulates
    cov_ut = Σ Wc_i · δ_i δ_iᵀ  with  δ_i = log(T_i · T_0⁻¹). Captures the
    wrong-convergence / initialisation uncertainty the analytic term ignores.
    Returns a 6×6 covariance in [rot|trans] order.
    """
    sigmas, Wc = _sigma_points(Q)

    Ts = []
    for xi in sigmas:
        T_init = _normalize_se3(_se3_exp(xi) @ center_T)
        res = refine(submap, full_map, T_init, gicp_params)
        Ts.append(np.asarray(res.transformation))

    T0_inv = np.linalg.inv(Ts[0])
    cov = np.zeros((6, 6))
    for i, T_i in enumerate(Ts):
        delta = _se3_log(T_i @ T0_inv)
        cov += Wc[i] * np.outer(delta, delta)
    return cov


def estimate_brossard(
    gicp_result: GicpResult,
    submap: o3d.geometry.PointCloud,
    full_map: o3d.geometry.PointCloud,
    params: BrossardParams = BrossardParams(),
    gicp_params: GicpParams = None,
) -> np.ndarray:
    """Brossard ICP covariance at the GICP-converged transform.

        use_ut = False:  cov = σ_sensor² · (Censi | Bonnabel)
        use_ut = True:   cov = σ_sensor² · (Censi | Bonnabel) + cov_ut(Q_ini)

    The analytic term comes from the icp_with_cov binary; the UT term re-runs
    GICP from sigma-point perturbations and so requires gicp_params.

    Returns a 6×6 covariance ordered [x, y, z, roll, pitch, yaw].
    """
    cov = _run_brossard_binary(gicp_result, submap, full_map, params)  # [rot|trans]

    if params.use_ut:
        if gicp_params is None:
            raise ValueError('use_ut=True requires gicp_params to re-run GICP')
        Q = _resolve_q_ini(params.q_ini_level)
        cov = cov + _compute_cov_ut(
            submap, full_map, gicp_result.transformation, Q, gicp_params)

    return _reorder_brossard_to_ours(cov)
