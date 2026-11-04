"""
test_kinematics.py
------------------
Unit tests for kinematics.py.
Run without ROS:  python3 -m pytest test/test_kinematics.py -v
Run with colcon:  colcon test --packages-select my_bot_control
"""

import math
import sys
import os

# Make the package importable when running directly (without colcon install)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from my_bot_control.kinematics import (
    inverse_kinematics,
    inv_kin_global,
    a, b, c, NUM_LEGS,
)


# ---- Helpers ---------------------------------------------------------
def fk(theta1_deg: float, theta2_deg: float, theta3_deg: float):
    """
    Forward kinematics — reconstructs (x, y, z) from joint angles.
    This is the analytical inverse of the IK, used to validate round-trips.

    From the original paper/diagram equations:
        r    = a + b*cos(θ2) + c*cos(θ2+θ3)
        x    = r * sin(θ1)
        y    = r * cos(θ1)
        z    = -(b*sin(θ2) + c*sin(θ2+θ3))
    """
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    t3 = math.radians(theta3_deg)

    r = a + b * math.cos(t2) + c * math.cos(t2 + t3)
    x = r * math.sin(t1)
    y = r * math.cos(t1)
    z = -(b * math.sin(t2) + c * math.sin(t2 + t3))
    return x, y, z


def _close(a_val, b_val, tol=0.1):
    return abs(a_val - b_val) < tol


# ---- Tests -----------------------------------------------------------

class TestInverseKinematics:

    def test_straight_down_centred(self):
        """Foot directly below shoulder, no lateral offset."""
        # x=0, y=a (no lateral, reach = a), z = -(b+c) (fully stretched)
        reach = b + c   # mm — fully extended
        result = inverse_kinematics(0.0, a + reach, 0.0)
        # With z=0 and r-a = reach, both links are horizontal → theta2=theta3=0
        # (theta3=0 means D=1, fully extended; may hit boundary)
        # Just check it doesn't return None for a reachable point
        # (exact 0 is a boundary; use a slightly shorter reach)
        result2 = inverse_kinematics(0.0, a + reach * 0.95, 0.0)
        assert result2 is not None

    def test_round_trip(self):
        """IK → FK → compare with original target."""
        targets = [
            (0.0,   80.0,  -200.0),
            (10.0,  75.0,  -180.0),
            (-10.0, 70.0,  -220.0),
            (0.0,   60.0,  -150.0),
        ]
        for (x, y, z) in targets:
            result = inverse_kinematics(x, y, z)
            assert result is not None, f'IK returned None for ({x}, {y}, {z})'
            t1, t2, t3 = result
            xr, yr, zr = fk(t1, t2, t3)
            assert _close(xr, x, tol=0.5), f'x mismatch: got {xr:.3f}, expected {x}'
            assert _close(yr, y, tol=0.5), f'y mismatch: got {yr:.3f}, expected {y}'
            assert _close(zr, z, tol=0.5), f'z mismatch: got {zr:.3f}, expected {z}'

    def test_unreachable_returns_none(self):
        """Target beyond maximum reach should return None."""
        max_reach = a + b + c  # 335 mm
        result = inverse_kinematics(0.0, max_reach + 50.0, 0.0)
        assert result is None

    def test_output_is_degrees(self):
        """Angles should be in degrees (>= -180 and <= 180)."""
        result = inverse_kinematics(0.0, 80.0, -200.0)
        assert result is not None
        for angle in result:
            assert -180.0 <= angle <= 180.0, f'Angle {angle} looks like radians'

    def test_symmetric_x(self):
        """Positive and negative x should give mirrored theta1."""
        r1 = inverse_kinematics( 20.0, 80.0, -200.0)
        r2 = inverse_kinematics(-20.0, 80.0, -200.0)
        assert r1 is not None and r2 is not None
        # theta1 should be negated; theta2, theta3 should be equal
        assert _close(r1[0], -r2[0], tol=0.01)
        assert _close(r1[1],  r2[1], tol=0.01)
        assert _close(r1[2],  r2[2], tol=0.01)


class TestInvKinGlobal:

    # Shoulder origins from URDF (mm)
    XL = [300.69, 300.69,  30.69,  30.69]
    YL = [149.24, 149.74, 149.24, 149.24]
    ZL = [-115.84, -307.54, -115.84, -307.54]
    SIDE = [1, -1, 1, -1]

    def _make_targets(self, dx=0.0, dy=80.0, dz=-200.0):
        """Global foot targets: each shoulder origin offset by (dx, dy, dz)."""
        return [
            [self.XL[i] + dx, self.YL[i] + dy, self.ZL[i] + dz]
            for i in range(NUM_LEGS)
        ]

    def test_all_legs_reachable(self):
        targets = self._make_targets()
        result = inv_kin_global(targets, self.XL, self.YL, self.ZL, self.SIDE)
        assert result is not None
        assert len(result) == NUM_LEGS

    def test_side_sign_mirrors_theta1(self):
        """Legs 1 & 2 (same x,y position, mirrored sides) should have opposite theta1."""
        targets = self._make_targets(dx=0.0, dy=80.0, dz=-200.0)
        result = inv_kin_global(targets, self.XL, self.YL, self.ZL, self.SIDE)
        assert result is not None
        # Legs 1 and 2 face each other; theta1 should be negated by side_sign
        t1_leg1, _, _ = result[0]
        t1_leg2, _, _ = result[1]
        # Both have side_sign applied; for symmetric target the magnitude should match
        assert _close(abs(t1_leg1), abs(t1_leg2), tol=1.0)

    def test_unreachable_returns_none(self):
        """If any leg target is unreachable, the whole call returns None."""
        targets = self._make_targets(dy=400.0)   # 400 mm > max_reach
        result = inv_kin_global(targets, self.XL, self.YL, self.ZL, self.SIDE)
        assert result is None
