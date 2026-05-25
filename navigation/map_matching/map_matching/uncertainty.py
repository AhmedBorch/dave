"""Uncertainty estimator for the GICP pose output.

Placeholder. Returns a diagonal 6x6 covariance whose magnitude is scaled
by the registration inlier RMSE and fitness. To be replaced with a
proper Hessian-based estimate using the GICP residuals/Jacobians.

Covariance order: [x, y, z, roll, pitch, yaw].
"""
from dataclasses import dataclass

import numpy as np

from .registration import GicpResult


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
    """Return a 6x6 diagonal covariance for the [x,y,z,roll,pitch,yaw] pose."""
    fitness = max(result.fitness, params.fitness_floor)
    rmse = max(result.inlier_rmse, 0.0)

    trans_var = (params.base_translation_var
                 + params.rmse_translation_gain * rmse * rmse) / fitness
    rot_var = (params.base_rotation_var
               + params.rmse_rotation_gain * rmse * rmse) / fitness

    trans_var = min(trans_var, params.max_translation_var)
    rot_var = min(rot_var, params.max_rotation_var)

    cov = np.zeros((6, 6))
    cov[0, 0] = cov[1, 1] = cov[2, 2] = trans_var
    cov[3, 3] = cov[4, 4] = cov[5, 5] = rot_var
    return cov
