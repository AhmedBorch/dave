"""Thrust allocation for the BlueROV2 Heavy (4 vectored + 2 vertical).

Inputs are four scalar body-frame commands (Newtons / Newton-metres, signed):
- surge : +x body force (forward positive)
- sway  : +y body force (port/left positive, FLU)
- yaw   : +z body torque (CCW from above positive)
- heave : +z body force (up positive)

Output: list of 6 thruster commands [T1..T6] in Newtons.

Allocation derived from the SDF thruster geometry (model.sdf):
  T1 @ (+0.14, -0.092)  thrust dir (-0.707, -0.707)   back-right
  T2 @ (+0.14, +0.092)  thrust dir (-0.707, +0.707)   back-left
  T3 @ (-0.15, -0.092)  thrust dir (+0.707, -0.707)   fwd-right
  T4 @ (-0.15, +0.092)  thrust dir (+0.707, +0.707)   fwd-left

Per-thruster contributions [Fx, Fy, Mz] (Mz = x*Fy - y*Fx):
        Fx       Fy       Mz
  T1  -0.707   -0.707   -0.164
  T2  -0.707   +0.707   +0.164
  T3  +0.707   -0.707   +0.171
  T4  +0.707   +0.707   -0.171

Inverting gives the decoupled command patterns (verified to first order):
  surge : (-1, -1, +1, +1)   pure +x,  zero Fy, zero Mz
  sway  : (-1, +1, -1, +1)   pure +y,  zero Fx, zero Mz
  yaw   : (-1, +1, +1, -1)   pure +Mz, zero Fx, zero Fy

NOTE: the previous version used yaw pattern (+1,-1,+1,-1), which is the NEGATIVE
of the sway pattern — i.e. commanding "yaw" actually produced sideways thrust
with almost no moment. That caused the vehicle to translate instead of rotate.
"""
from dataclasses import dataclass
from typing import List


@dataclass
class ThrustLimits:
    max_thrust: float = 4.0   # N per thruster, hard clip


def _clip(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def allocate(surge: float, sway: float, yaw: float, heave: float,
             limits: ThrustLimits = ThrustLimits()) -> List[float]:
    """Mix the four body-frame commands into per-thruster commands."""
    t1 = -surge - sway - yaw
    t2 = -surge + sway + yaw
    t3 = +surge - sway + yaw
    t4 = +surge + sway - yaw
    t5 = -heave
    t6 = -heave
    return [_clip(t, limits.max_thrust) for t in (t1, t2, t3, t4, t5, t6)]
