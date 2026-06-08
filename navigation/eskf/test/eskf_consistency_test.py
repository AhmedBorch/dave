"""ESKF consistency evaluation node.

Publishes live RMSE, NEES and NIS metrics with 95% chi-squared bounds so you
can judge filter consistency from rqt_plot or PlotJuggler without post-processing.

Consistency rules of thumb
--------------------------
NEES (Normalised Estimation Error Squared)
  - Single-sample NES_pos ~ chi2(3):  95% in [0.22, 9.35]
  - Average NEES converges to n=3 for a consistent filter.
    Bounds shrink as N grows — published as dynamic lower/upper.
  - NEES >> upper  → filter OVERCONFIDENT (P too small)
  - NEES << lower  → filter UNDERCONFIDENT (P too large)

NIS (Normalised Innovation Squared) — read from ESKF nis topics
  - DVL   (3-DOF): 95% chi2(3)  → [0.22, 9.35]
  - Depth (1-DOF): 95% chi2(1)  → [0.00, 5.02]

Topics consumed
---------------
  /model/bluerov2/eskf/odom   — ESKF estimate with covariance
  /model/bluerov2/odometry    — Gazebo ground truth
  eskf/nis_dvl                — DVL   NIS (Float64)
  eskf/nis_depth              — Depth NIS (Float64)

All published as std_msgs/Float64.
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import chi2

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ---------------------------------------------------------------------------
EST_TOPIC       = '/model/bluerov2/eskf/odom'
GT_TOPIC        = '/model/bluerov2/odometry'
NIS_DVL_TOPIC   = 'eskf/nis_dvl'
NIS_DEPTH_TOPIC = 'eskf/nis_depth'

EVAL_RATE_HZ  = 10.0   # how often to compare latest messages
PRINT_RATE_HZ = 0.2    # how often to print summary to terminal

DOF_POS   = 3
DOF_ORI   = 3
DOF_POSE  = 6   # joint position + orientation
DOF_VEL   = 3
DOF_DVL   = 3
DOF_DEPTH = 1


def _chi2_bounds(dof):
    return chi2.ppf(0.025, dof), chi2.ppf(0.975, dof)


CHI2_POS_LO,   CHI2_POS_HI   = _chi2_bounds(DOF_POS)
CHI2_ORI_LO,   CHI2_ORI_HI   = _chi2_bounds(DOF_ORI)
CHI2_POSE_LO,  CHI2_POSE_HI  = _chi2_bounds(DOF_POSE)
CHI2_VEL_LO,   CHI2_VEL_HI   = _chi2_bounds(DOF_VEL)
CHI2_DVL_LO,   CHI2_DVL_HI   = _chi2_bounds(DOF_DVL)
CHI2_DEPTH_LO, CHI2_DEPTH_HI = _chi2_bounds(DOF_DEPTH)


def _avg_chi2_bounds(dof, n):
    """95% bounds for the average of n iid chi2(dof) samples."""
    if n < 1:
        return 0.0, float('inf')
    return chi2.ppf(0.025, dof * n) / n, chi2.ppf(0.975, dof * n) / n


def _safe_nees(err, cov):
    try:
        return float(err @ np.linalg.inv(cov) @ err)
    except np.linalg.LinAlgError:
        return None


class EskfConsistencyNode(Node):
    def __init__(self):
        super().__init__('eskf_consistency')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Latest messages — no synchronizer, avoids wall-clock vs sim-time mismatch
        self._last_est: Odometry | None = None
        self._last_gt:  Odometry | None = None

        self.create_subscription(Odometry, EST_TOPIC, self._on_est, qos)
        self.create_subscription(Odometry, GT_TOPIC,  self._on_gt,  qos)
        self.create_subscription(Float64, NIS_DVL_TOPIC,   self._on_nis_dvl,   qos)
        self.create_subscription(Float64, NIS_DEPTH_TOPIC, self._on_nis_depth, qos)

        # --- Accumulators ---
        self._n = 0
        self._sse_pos   = np.zeros(3)
        self._sse_vel   = np.zeros(3)
        self._sse_pos3d = 0.0
        self._sse_vel3d = 0.0
        self._sse_ori   = 0.0

        self._nees_pos_sum  = 0.0
        self._nees_ori_sum  = 0.0
        self._nees_pose_sum = 0.0   # joint 6-DOF (pos + ori)
        self._nees_vel_sum  = 0.0

        self._n_dvl = 0;   self._nis_dvl_sum   = 0.0
        self._n_dep = 0;   self._nis_dep_sum   = 0.0

        # --- Publishers ---
        def p(t): return self.create_publisher(Float64, t, 10)

        # RMSE
        self._pub = {
            'rmse_px':  p('eskf_metrics/rmse/position/x'),
            'rmse_py':  p('eskf_metrics/rmse/position/y'),
            'rmse_pz':  p('eskf_metrics/rmse/position/z'),
            'rmse_p3d': p('eskf_metrics/rmse/position/total'),
            'rmse_vx':  p('eskf_metrics/rmse/velocity/x'),
            'rmse_vy':  p('eskf_metrics/rmse/velocity/y'),
            'rmse_vz':  p('eskf_metrics/rmse/velocity/z'),
            'rmse_v3d': p('eskf_metrics/rmse/velocity/total'),
            'rmse_ori': p('eskf_metrics/rmse/orientation_rad'),
            # NEES — position (3-DOF, marginal)
            'nees_p':      p('eskf_metrics/nees/position'),
            'nees_p_avg':  p('eskf_metrics/nees/position/avg'),
            'nees_p_lo':   p('eskf_metrics/nees/position/lower95'),
            'nees_p_hi':   p('eskf_metrics/nees/position/upper95'),
            'nees_p_lo_s': p('eskf_metrics/nees/position/lower95_single'),
            'nees_p_hi_s': p('eskf_metrics/nees/position/upper95_single'),
            # NEES — orientation (3-DOF, marginal)
            'nees_o':      p('eskf_metrics/nees/orientation'),
            'nees_o_avg':  p('eskf_metrics/nees/orientation/avg'),
            'nees_o_lo':   p('eskf_metrics/nees/orientation/lower95'),
            'nees_o_hi':   p('eskf_metrics/nees/orientation/upper95'),
            'nees_o_lo_s': p('eskf_metrics/nees/orientation/lower95_single'),
            'nees_o_hi_s': p('eskf_metrics/nees/orientation/upper95_single'),
            # NEES — joint pose (6-DOF, uses full 6×6 covariance with cross terms)
            'nees_pose':      p('eskf_metrics/nees/pose'),
            'nees_pose_avg':  p('eskf_metrics/nees/pose/avg'),
            'nees_pose_lo':   p('eskf_metrics/nees/pose/lower95'),
            'nees_pose_hi':   p('eskf_metrics/nees/pose/upper95'),
            'nees_pose_lo_s': p('eskf_metrics/nees/pose/lower95_single'),
            'nees_pose_hi_s': p('eskf_metrics/nees/pose/upper95_single'),
            # NEES — velocity (3-DOF)
            'nees_v':      p('eskf_metrics/nees/velocity'),
            'nees_v_avg':  p('eskf_metrics/nees/velocity/avg'),
            'nees_v_lo':   p('eskf_metrics/nees/velocity/lower95'),       # dynamic avg bound
            'nees_v_hi':   p('eskf_metrics/nees/velocity/upper95'),       # dynamic avg bound
            'nees_v_lo_s': p('eskf_metrics/nees/velocity/lower95_single'),# constant
            'nees_v_hi_s': p('eskf_metrics/nees/velocity/upper95_single'),# constant
            # NIS — DVL
            'nis_dvl':        p('eskf_metrics/nis/dvl'),
            'nis_dvl_avg':    p('eskf_metrics/nis/dvl/avg'),
            'nis_dvl_lo':     p('eskf_metrics/nis/dvl/lower95'),
            'nis_dvl_hi':     p('eskf_metrics/nis/dvl/upper95'),
            'nis_dvl_avg_lo': p('eskf_metrics/nis/dvl/avg_lower95'),
            'nis_dvl_avg_hi': p('eskf_metrics/nis/dvl/avg_upper95'),
            # NIS — depth
            'nis_dep':        p('eskf_metrics/nis/depth'),
            'nis_dep_avg':    p('eskf_metrics/nis/depth/avg'),
            'nis_dep_lo':     p('eskf_metrics/nis/depth/lower95'),
            'nis_dep_hi':     p('eskf_metrics/nis/depth/upper95'),
            'nis_dep_avg_lo': p('eskf_metrics/nis/depth/avg_lower95'),
            'nis_dep_avg_hi': p('eskf_metrics/nis/depth/avg_upper95'),
            # Overall flag
            'consistent': p('eskf_metrics/consistent'),
        }
        self._euler_est_pub = self.create_publisher(Float64MultiArray, 'debug/euler/est', 10)
        self._euler_gt_pub  = self.create_publisher(Float64MultiArray, 'debug/euler/gt',  10)

        self.create_timer(1.0 / EVAL_RATE_HZ,  self._evaluate)
        self.create_timer(1.0 / PRINT_RATE_HZ, self._print_summary)

        self.get_logger().info(
            f'ESKF consistency node ready.\n'
            f'  chi2(3) single-sample 95%% bounds: [{CHI2_POS_LO:.3f}, {CHI2_POS_HI:.3f}]\n'
            f'  chi2(1) single-sample 95%% bounds: [{CHI2_DEPTH_LO:.3f}, {CHI2_DEPTH_HI:.3f}]'
        )

    # ------------------------------------------------------------------ subscribers

    def _on_est(self, msg): self._last_est = msg
    def _on_gt(self,  msg): self._last_gt  = msg

    def _on_nis_dvl(self, msg):
        v = msg.data
        self._n_dvl += 1
        self._nis_dvl_sum += v
        avg = self._nis_dvl_sum / self._n_dvl
        lo_d, hi_d = _avg_chi2_bounds(DOF_DVL, self._n_dvl)
        self._publish('nis_dvl',        v)
        self._publish('nis_dvl_avg',    avg)
        self._publish('nis_dvl_lo',     CHI2_DVL_LO)
        self._publish('nis_dvl_hi',     CHI2_DVL_HI)
        self._publish('nis_dvl_avg_lo', lo_d)
        self._publish('nis_dvl_avg_hi', hi_d)

    def _on_nis_depth(self, msg):
        v = msg.data
        self._n_dep += 1
        self._nis_dep_sum += v
        avg = self._nis_dep_sum / self._n_dep
        lo_d, hi_d = _avg_chi2_bounds(DOF_DEPTH, self._n_dep)
        self._publish('nis_dep',        v)
        self._publish('nis_dep_avg',    avg)
        self._publish('nis_dep_lo',     CHI2_DEPTH_LO)
        self._publish('nis_dep_hi',     CHI2_DEPTH_HI)
        self._publish('nis_dep_avg_lo', lo_d)
        self._publish('nis_dep_avg_hi', hi_d)

    # ------------------------------------------------------------------ evaluation

    def _publish(self, key, value):
        self._pub[key].publish(Float64(data=float(value)))

    def _evaluate(self):
        if self._last_est is None or self._last_gt is None:
            return

        est, gt = self._last_est, self._last_gt
        self._n += 1
        n = self._n

        # Position
        p_e = np.array([est.pose.pose.position.x, est.pose.pose.position.y, est.pose.pose.position.z])
        p_g = np.array([gt.pose.pose.position.x,  gt.pose.pose.position.y,  gt.pose.pose.position.z])
        ep  = p_e - p_g
        self._sse_pos   += ep ** 2
        self._sse_pos3d += float(ep @ ep)

        self._publish('rmse_px',  math.sqrt(self._sse_pos[0] / n))
        self._publish('rmse_py',  math.sqrt(self._sse_pos[1] / n))
        self._publish('rmse_pz',  math.sqrt(self._sse_pos[2] / n))
        self._publish('rmse_p3d', math.sqrt(self._sse_pos3d  / n))

        # Velocity
        v_e = np.array([est.twist.twist.linear.x, est.twist.twist.linear.y, est.twist.twist.linear.z])
        v_g = np.array([gt.twist.twist.linear.x,  gt.twist.twist.linear.y,  gt.twist.twist.linear.z])
        ev  = v_e - v_g
        self._sse_vel   += ev ** 2
        self._sse_vel3d += float(ev @ ev)

        self._publish('rmse_vx',  math.sqrt(self._sse_vel[0] / n))
        self._publish('rmse_vy',  math.sqrt(self._sse_vel[1] / n))
        self._publish('rmse_vz',  math.sqrt(self._sse_vel[2] / n))
        self._publish('rmse_v3d', math.sqrt(self._sse_vel3d  / n))

        # Orientation (geodesic rotation-vector error)
        def _q(o): return [o.x, o.y, o.z, o.w]
        r_e = Rotation.from_quat(_q(est.pose.pose.orientation))
        r_g = Rotation.from_quat(_q(gt.pose.pose.orientation))
        eo  = (r_g.inv() * r_e).as_rotvec()
        self._sse_ori += float(eo @ eo)
        self._publish('rmse_ori', math.sqrt(self._sse_ori / n))

        euler_est = r_e.as_euler('xyz', degrees=True)
        euler_gt  = r_g.as_euler('xyz', degrees=True)
        msg_e = Float64MultiArray(); msg_e.data = euler_est.tolist()
        msg_g = Float64MultiArray(); msg_g.data = euler_gt.tolist()
        self._euler_est_pub.publish(msg_e)
        self._euler_gt_pub.publish(msg_g)

        # Full 6×6 pose covariance — includes pos-att cross terms now that
        # the ESKF publishes them.  Layout: [x,y,z,rx,ry,rz]
        cov6  = np.array(est.pose.covariance).reshape(6, 6)
        cov_p = cov6[0:3, 0:3]   # position marginal
        cov_o = cov6[3:6, 3:6]   # orientation marginal
        cov_t = np.array(est.twist.covariance).reshape(6, 6)
        cov_v = cov_t[0:3, 0:3]  # velocity

        # Joint 6-D error and full covariance for combined pose NEES
        e_pose = np.concatenate([ep, eo])   # [δp; δθ]  shape (6,)

        # NEES — position (3-DOF, marginal)
        nees_p = _safe_nees(ep, cov_p)
        if nees_p is not None:
            self._nees_pos_sum += nees_p
            avg = self._nees_pos_sum / n
            lo_d, hi_d = _avg_chi2_bounds(DOF_POS, n)
            self._publish('nees_p',      nees_p)
            self._publish('nees_p_avg',  avg)
            self._publish('nees_p_lo',   lo_d)
            self._publish('nees_p_hi',   hi_d)
            self._publish('nees_p_lo_s', CHI2_POS_LO)
            self._publish('nees_p_hi_s', CHI2_POS_HI)

        # NEES — orientation (3-DOF, marginal)
        nees_o = _safe_nees(eo, cov_o)
        if nees_o is not None:
            self._nees_ori_sum += nees_o
            avg = self._nees_ori_sum / n
            lo_d, hi_d = _avg_chi2_bounds(DOF_ORI, n)
            self._publish('nees_o',      nees_o)
            self._publish('nees_o_avg',  avg)
            self._publish('nees_o_lo',   lo_d)
            self._publish('nees_o_hi',   hi_d)
            self._publish('nees_o_lo_s', CHI2_ORI_LO)
            self._publish('nees_o_hi_s', CHI2_ORI_HI)

        # NEES — joint pose (6-DOF) using full 6×6 covariance with cross terms
        nees_pose = _safe_nees(e_pose, cov6)
        if nees_pose is not None:
            self._nees_pose_sum += nees_pose
            avg = self._nees_pose_sum / n
            lo_d, hi_d = _avg_chi2_bounds(DOF_POSE, n)
            self._publish('nees_pose',      nees_pose)
            self._publish('nees_pose_avg',  avg)
            self._publish('nees_pose_lo',   lo_d)
            self._publish('nees_pose_hi',   hi_d)
            self._publish('nees_pose_lo_s', CHI2_POSE_LO)
            self._publish('nees_pose_hi_s', CHI2_POSE_HI)
            self._publish('consistent',     1.0 if lo_d <= avg <= hi_d else 0.0)

        # NEES — velocity (3-DOF)
        nees_v = _safe_nees(ev, cov_v)
        if nees_v is not None:
            self._nees_vel_sum += nees_v
            avg = self._nees_vel_sum / n
            lo_d, hi_d = _avg_chi2_bounds(DOF_VEL, n)
            self._publish('nees_v',     nees_v)
            self._publish('nees_v_avg', avg)
            self._publish('nees_v_lo',  lo_d)   # dynamic average bound
            self._publish('nees_v_hi',  hi_d)

        # Always publish all constant chi² bounds so they appear in rqt_plot /
        # PlotJuggler even before enough samples have been collected.
        # NEES single-sample bounds
        self._publish('nees_p_lo_s',    CHI2_POS_LO)
        self._publish('nees_p_hi_s',    CHI2_POS_HI)
        self._publish('nees_o_lo_s',    CHI2_ORI_LO)
        self._publish('nees_o_hi_s',    CHI2_ORI_HI)
        self._publish('nees_pose_lo_s', CHI2_POSE_LO)
        self._publish('nees_pose_hi_s', CHI2_POSE_HI)
        self._publish('nees_v_lo_s',    CHI2_VEL_LO)
        self._publish('nees_v_hi_s',    CHI2_VEL_HI)
        # NIS single-sample bounds
        self._publish('nis_dvl_lo',   CHI2_DVL_LO)
        self._publish('nis_dvl_hi',   CHI2_DVL_HI)
        self._publish('nis_dep_lo',   CHI2_DEPTH_LO)
        self._publish('nis_dep_hi',   CHI2_DEPTH_HI)
        # Dynamic average NIS bounds (based on samples received so far)
        if self._n_dvl > 0:
            lo_d, hi_d = _avg_chi2_bounds(DOF_DVL, self._n_dvl)
            self._publish('nis_dvl_avg_lo', lo_d)
            self._publish('nis_dvl_avg_hi', hi_d)
        if self._n_dep > 0:
            lo_d, hi_d = _avg_chi2_bounds(DOF_DEPTH, self._n_dep)
            self._publish('nis_dep_avg_lo', lo_d)
            self._publish('nis_dep_avg_hi', hi_d)

    # ------------------------------------------------------------------ terminal summary

    def _print_summary(self):
        n = self._n
        if n == 0:
            missing = []
            if self._last_est is None: missing.append(EST_TOPIC)
            if self._last_gt  is None: missing.append(GT_TOPIC)
            if missing:
                self.get_logger().warn(f'Waiting for: {missing}')
            return

        def _avg_nees(s): return s / n if n > 0 else float('nan')
        lo_p, hi_p = _avg_chi2_bounds(DOF_POS, n)
        avg_p = _avg_nees(self._nees_pos_sum)
        flag  = 'OK' if lo_p <= avg_p <= hi_p else ('OVERCONFIDENT' if avg_p > hi_p else 'UNDERCONFIDENT')

        rmse_p = math.sqrt(self._sse_pos3d / n) if n > 0 else 0.0
        rmse_o = math.sqrt(self._sse_ori   / n) if n > 0 else 0.0

        self.get_logger().info(
            f'[n={n}] RMSE_pos={rmse_p:.3f}m  RMSE_ori={math.degrees(rmse_o):.2f}°  '
            f'ANEES_pos={avg_p:.2f} [{lo_p:.2f},{hi_p:.2f}] → {flag}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = EskfConsistencyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
