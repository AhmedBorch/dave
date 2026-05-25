"""Smoke test for the registration pipeline on synthetic data."""
import numpy as np
import open3d as o3d

from map_matching.pipeline import PipelineParams, run
from map_matching.ransac import RansacParams
from map_matching.registration import GicpParams


def _make_box_cloud(n: int = 5000, size: float = 2.0) -> o3d.geometry.PointCloud:
    rng = np.random.default_rng(0)
    pts = rng.uniform(-size / 2, size / 2, size=(n, 3))
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(pts)
    return cloud


def test_pipeline_runs_on_synthetic_clouds():
    target = _make_box_cloud()
    # source = target rotated 10 deg about z and translated by [0.5, 0.2, 0]
    angle = np.deg2rad(10.0)
    R = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle),  np.cos(angle), 0],
        [0, 0, 1],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [0.5, 0.2, 0.0]

    source = o3d.geometry.PointCloud(target)
    source.transform(np.linalg.inv(T))

    params = PipelineParams(
        ransac=RansacParams(voxel_size=0.1, max_iterations=10_000),
        gicp=GicpParams(max_correspondence_distance=0.3, voxel_size=0.05),
    )

    result = run(source, target, T_world_struct=None, params=params)

    assert result.pose_world.shape == (4, 4)
    assert result.covariance.shape == (6, 6)
    assert 0.0 <= result.gicp.fitness <= 1.0
