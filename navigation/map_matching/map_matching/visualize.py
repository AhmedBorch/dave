"""Standalone debug viewer for the map-matching pipeline.

Loads a submap PCD and a full-map PCD, runs RANSAC then GICP, and opens
an Open3D window showing:
  - full map (target)                      : grey
  - submap transformed by RANSAC           : blue
  - submap transformed by GICP (refinement): red

Usage:
    python3 src/dave/navigation/map_matching/map_matching/visualize.py SUBMAP.pcd FULLMAP.pcd
"""
import argparse
import copy
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

# Allow running directly as a script (without `colcon build`):
# add the package root (parent of this file's dir) to sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from map_matching.ransac import RansacParams, align          # noqa: E402
from map_matching.registration import GicpParams, refine     # noqa: E402


GREY = [0.6, 0.6, 0.6]
BLUE = [0.10, 0.55, 0.95]    # RANSAC result
RED  = [0.95, 0.20, 0.20]    # GICP result


def _color(cloud: o3d.geometry.PointCloud, rgb) -> o3d.geometry.PointCloud:
    out = copy.deepcopy(cloud)
    out.paint_uniform_color(rgb)
    return out


def _sanitize(cloud: o3d.geometry.PointCloud,
              max_range: float) -> o3d.geometry.PointCloud:
    """Drop NaN/Inf points and clip points whose range exceeds max_range."""
    cloud = cloud.remove_non_finite_points()
    pts = np.asarray(cloud.points)
    if pts.size == 0:
        return cloud
    ranges = np.linalg.norm(pts, axis=1)
    mask = ranges <= max_range
    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(pts[mask])
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('submap', help='Path to submap (source) PCD.')
    parser.add_argument('full_map', help='Path to full map (target) PCD.')
    parser.add_argument('--ransac-voxel', type=float, default=0.03)
    parser.add_argument('--gicp-voxel', type=float, default=0.03)
    parser.add_argument('--gicp-max-dist', type=float, default=0.5)
    parser.add_argument('--submap-max-range', type=float, default=20.0,
                        help='Drop submap points beyond this range from origin (m).')
    parser.add_argument('--map-max-range', type=float, default=100.0,
                        help='Drop full-map points beyond this range from origin (m).')
    parser.add_argument('--no-window', action='store_true',
                        help='Skip the GUI; just print fitness/RMSE.')
    args = parser.parse_args(argv)

    submap = o3d.io.read_point_cloud(args.submap)
    full_map = o3d.io.read_point_cloud(args.full_map)
    if len(submap.points) == 0 or len(full_map.points) == 0:
        print('Error: one of the clouds is empty.', file=sys.stderr)
        return 1
    print(f'submap   raw: {len(submap.points)} points')
    print(f'full map raw: {len(full_map.points)} points')

    submap = _sanitize(submap, args.submap_max_range)
    full_map = _sanitize(full_map, args.map_max_range)
    print(f'submap   clean: {len(submap.points)} points  (range <= {args.submap_max_range} m)')
    print(f'full map clean: {len(full_map.points)} points  (range <= {args.map_max_range} m)')
    if len(submap.points) == 0 or len(full_map.points) == 0:
        print('Error: a cloud became empty after sanitization. Increase --*-max-range.',
              file=sys.stderr)
        return 1

    ransac_params = RansacParams(voxel_size=args.ransac_voxel)
    gicp_params = GicpParams(voxel_size=args.gicp_voxel,
                             max_correspondence_distance=args.gicp_max_dist)

    print('\n[1/2] RANSAC global alignment...')
    T_coarse = align(submap, full_map, ransac_params)
    print(f'  T_coarse =\n{np.array2string(T_coarse, precision=3, suppress_small=True)}')

    print('\n[2/2] GICP refinement...')
    gicp_res = refine(submap, full_map, T_coarse, gicp_params)
    print(f'  fitness    = {gicp_res.fitness:.3f}')
    print(f'  inlier RMSE= {gicp_res.inlier_rmse:.4f} m')
    print(f'  T_refined  =\n{np.array2string(gicp_res.transformation, precision=3, suppress_small=True)}')

    if args.no_window:
        return 0

    target = _color(full_map, GREY)
    ransac_cloud = _color(submap, BLUE).transform(T_coarse)
    gicp_cloud = _color(submap, RED).transform(gicp_res.transformation)

    print('\nLegend: grey=full map | blue=RANSAC | red=GICP. Close the window to exit.')
    o3d.visualization.draw_geometries(
        [target, ransac_cloud, gicp_cloud],
        window_name='map_matching: RANSAC (blue) vs GICP (red) on full map (grey)',
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
