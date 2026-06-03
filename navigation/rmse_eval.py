#!/usr/bin/env python3
"""
Live RMSE comparison: ESKF without map-matching vs ESKF with map-matching,
both evaluated against Gazebo ground-truth odometry.

Usage
-----
  python3 rmse_eval.py

Topics (edit the constants below if needed)
-------------------------------------------
  TOPIC_GT      ground-truth odometry (Gazebo)
  TOPIC_BASE    ESKF without map-matcher  (eskf_base from comparison launch)
  TOPIC_MAP     ESKF with map-matcher     (eskf_map  from comparison launch)

Published RMSE topics (std_msgs/Float64, updated every PRINT_INTERVAL s)
-------------------------------------------------------------------------
  ~/rmse/base/{x,y,z,xy,3d,roll,pitch,yaw}
  ~/rmse/map/{x,y,z,xy,3d,roll,pitch,yaw}

Launch the comparison stack first:
  ros2 launch eskf eskf_comparison.launch.py
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float64

# ---------------------------------------------------------------------------
TOPIC_GT   = '/model/bluerov2/odometry'
TOPIC_BASE = '/model/bluerov2/eskf/odom'
TOPIC_MAP  = '/model/bluerov2/eskf_map/odom'

PRINT_INTERVAL = 5.0   # seconds between table prints and topic publishes
# ---------------------------------------------------------------------------


def _yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _pitch(q) -> float:
    return math.asin(float(np.clip(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)))


def _roll(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    )


def _wrap(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


def _pos(msg) -> np.ndarray:
    p = msg.pose.pose.position
    return np.array([p.x, p.y, p.z])


class RMSEEvaluator(Node):
    def __init__(self):
        super().__init__('rmse_evaluator')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Latest message from each topic
        self._last_gt:   Odometry | None = None
        self._last_base: Odometry | None = None
        self._last_map:  Odometry | None = None

        self.create_subscription(Odometry, TOPIC_GT,   self._on_gt,   qos)
        self.create_subscription(Odometry, TOPIC_BASE, self._on_base, qos)
        self.create_subscription(Odometry, TOPIC_MAP,  self._on_map,  qos)

        # Accumulators (sum of squared errors)
        self._n: int = 0
        self._base_sse = np.zeros(3)          # x, y, z
        self._map_sse  = np.zeros(3)
        self._base_rpy_sse = np.zeros(3)      # roll, pitch, yaw
        self._map_rpy_sse  = np.zeros(3)

        # RMSE publishers — one Float64 per metric per instance
        def _pub(topic): return self.create_publisher(Float64, topic, 10)
        self._pubs_base = {
            'x':     _pub('~/rmse/base/x'),
            'y':     _pub('~/rmse/base/y'),
            'z':     _pub('~/rmse/base/z'),
            'xy':    _pub('~/rmse/base/xy'),
            '3d':    _pub('~/rmse/base/rmse_3d'),
            'roll':  _pub('~/rmse/base/roll'),
            'pitch': _pub('~/rmse/base/pitch'),
            'yaw':   _pub('~/rmse/base/yaw'),
        }
        self._pubs_map = {
            'x':     _pub('~/rmse/map/x'),
            'y':     _pub('~/rmse/map/y'),
            'z':     _pub('~/rmse/map/z'),
            'xy':    _pub('~/rmse/map/xy'),
            '3d':    _pub('~/rmse/map/rmse_3d'),
            'roll':  _pub('~/rmse/map/roll'),
            'pitch': _pub('~/rmse/map/pitch'),
            'yaw':   _pub('~/rmse/map/yaw'),
        }

        self.create_timer(0.1,            self._evaluate)   # 10 Hz
        self.create_timer(PRINT_INTERVAL, self._print)

        self.get_logger().info(
            f'Listening:\n'
            f'  GT   → {TOPIC_GT}\n'
            f'  BASE → {TOPIC_BASE}\n'
            f'  MAP  → {TOPIC_MAP}\n'
            f'Printing and publishing RMSE every {PRINT_INTERVAL:.0f} s  '
            f'|  Ctrl-C for final summary'
        )

    # ------------------------------------------------------------------ callbacks

    def _on_gt(self,   msg: Odometry) -> None: self._last_gt   = msg
    def _on_base(self, msg: Odometry) -> None: self._last_base = msg
    def _on_map(self,  msg: Odometry) -> None: self._last_map  = msg

    # ------------------------------------------------------------------ evaluation

    def _evaluate(self) -> None:
        if self._last_gt is None or self._last_base is None or self._last_map is None:
            return

        gt = self._last_gt
        gt_q = gt.pose.pose.orientation

        self._n += 1

        self._base_sse += (_pos(self._last_base) - _pos(gt)) ** 2
        self._map_sse  += (_pos(self._last_map)  - _pos(gt)) ** 2

        base_q = self._last_base.pose.pose.orientation
        map_q  = self._last_map.pose.pose.orientation

        self._base_rpy_sse += np.array([
            _wrap(_roll(base_q)  - _roll(gt_q)),
            _wrap(_pitch(base_q) - _pitch(gt_q)),
            _wrap(_yaw(base_q)   - _yaw(gt_q)),
        ]) ** 2
        self._map_rpy_sse += np.array([
            _wrap(_roll(map_q)  - _roll(gt_q)),
            _wrap(_pitch(map_q) - _pitch(gt_q)),
            _wrap(_yaw(map_q)   - _yaw(gt_q)),
        ]) ** 2

    # ------------------------------------------------------------------ printing & publishing

    def _compute_rmse(self):
        n = self._n
        base_rmse = np.sqrt(self._base_sse / n)
        map_rmse  = np.sqrt(self._map_sse  / n)
        base_rpy  = np.sqrt(self._base_rpy_sse / n)
        map_rpy   = np.sqrt(self._map_rpy_sse  / n)
        base_xy   = math.sqrt(float(np.sum(self._base_sse[:2])) / n)
        map_xy    = math.sqrt(float(np.sum(self._map_sse[:2]))  / n)
        base_3d   = math.sqrt(float(np.sum(self._base_sse)) / n)
        map_3d    = math.sqrt(float(np.sum(self._map_sse))  / n)
        return {
            'x':     (base_rmse[0], map_rmse[0]),
            'y':     (base_rmse[1], map_rmse[1]),
            'z':     (base_rmse[2], map_rmse[2]),
            'xy':    (base_xy,      map_xy),
            '3d':    (base_3d,      map_3d),
            'roll':  (base_rpy[0],  map_rpy[0]),
            'pitch': (base_rpy[1],  map_rpy[1]),
            'yaw':   (base_rpy[2],  map_rpy[2]),
        }

    def _publish_rmse(self, rmse: dict) -> None:
        for key, (b, m) in rmse.items():
            self._pubs_base[key].publish(Float64(data=b))
            self._pubs_map[key].publish(Float64(data=m))

    def _print(self) -> None:
        gt_ok   = self._last_gt   is not None
        base_ok = self._last_base is not None
        map_ok  = self._last_map  is not None

        if not (gt_ok and base_ok and map_ok):
            missing = [t for t, ok in [
                (TOPIC_GT, gt_ok), (TOPIC_BASE, base_ok), (TOPIC_MAP, map_ok)
            ] if not ok]
            print(f'[rmse_eval] Waiting for: {missing}')
            return

        n = self._n
        if n == 0:
            print('[rmse_eval] Topics connected — accumulating samples...')
            return

        rmse = self._compute_rmse()
        self._publish_rmse(rmse)

        def pct(b, m):
            return f'({(b - m) / b * 100:+.1f}%)' if b > 1e-9 else ''

        W = 14
        rows = [
            ('x  (m)',     'x'),
            ('y  (m)',     'y'),
            ('z  (m)',     'z'),
            ('xy (m)',     'xy'),
            ('3D pos (m)', '3d'),
            ('roll  (rad)', 'roll'),
            ('pitch (rad)', 'pitch'),
            ('yaw   (rad)', 'yaw'),
        ]

        print()
        print('=' * 64)
        print(f'  RMSE  (n={n} samples)')
        print('=' * 64)
        print(f'  {"Metric":<22} {"ESKF base":>{W}} {"ESKF+map":>{W}}')
        print(f'  {"-"*60}')
        for label, key in rows:
            b, m = rmse[key]
            print(f'  {label:<22} {b:>{W}.4f} {m:>{W}.4f}  {pct(b, m)}')
        print('=' * 64)
        print('  +% = map-aided is better')
        print()


def main():
    rclpy.init()
    node = RMSEEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[rmse_eval] Final results:')
        node._print()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
