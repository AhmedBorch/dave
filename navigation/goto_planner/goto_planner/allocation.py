"""Simple thrust allocation for the BlueROV2 (4 vectored + 2 vertical).

Inputs are three scalar commands (Newtons, signed):
- surge   : +x body force (forward positive)
- yaw     : +z body torque (CCW from above positive)
- heave   : +z body force (up positive)

Output: list of 6 thruster commands [T1..T6] in Newtons.

Allocation patterns are taken from the working primitives in
`move_forward.py` and `move_down.py` and extended to yaw by symmetry.
The geometry is not perfectly square so per-axis gains may need tuning,
but the patterns themselves give pure (decoupled) motion to first order.
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


def allocate(surge: float, yaw: float, heave: float,
             limits: ThrustLimits = ThrustLimits()) -> List[float]:
    """Mix the three body-frame commands into per-thruster commands."""
    t1 = -surge + yaw
    t2 = -surge - yaw
    t3 = +surge + yaw
    t4 = +surge - yaw
    t5 = -heave
    t6 = -heave
    return [_clip(t, limits.max_thrust) for t in (t1, t2, t3, t4, t5, t6)]
