"""
kinematics.py
-------------
Direct Python port of Kinematics.cpp + RobotConfig.cpp.

Link lengths (from RobotConfig.cpp, units: mm):
    a = 60.0   shoulder offset
    b = 95.0   femur / upper leg
    c = 180.0  tibia / lower leg

inverse_kinematics()  →  port of my_bot_ik::inverseKinematics()
inv_kin_global()      →  port of my_bot_ik::inv_kin_global()

Both functions return angles in DEGREES, exactly as the C++ original does,
so the output is directly comparable to the ESP servo values.

NOTE for Gazebo use:
    ROS 2 joint position controllers expect RADIANS.
    Convert with:  angle_rad = math.radians(angle_deg)
    The IK solver node (ik_solver_node.py) performs this conversion.

No ROS imports here — this file is pure math and can be unit-tested
without a running ROS 2 environment.
"""

import math

# ---- Robot configuration (RobotConfig.cpp) ---------------------------
NUM_LEGS   = 4
NUM_JOINTS = 3   # shoulder, femur, leg (wheel is not IK-controlled)

a: float = 60.0   # shoulder link  [mm]
b: float = 95.0   # femur link     [mm]
c: float = 180.0  # tibia link     [mm]

_RAD_TO_DEG = 180.0 / math.pi


# ---- inverseKinematics() ---------------------------------------------
def inverse_kinematics(
    x: float,
    y: float,
    z: float,
) -> tuple[float, float, float] | None:
    """
    Port of my_bot_ik::inverseKinematics().

    Solves IK for a single leg in the local leg frame.

    Args:
        x, y, z : foot position in the local leg frame [mm].
                  x   — lateral deviation
                  y   — forward reach in the leg plane
                  z   — vertical (negative = down)

    Returns:
        (theta1_deg, theta2_deg, theta3_deg) on success.
        None if the target is out of reach.

    Angle convention (matches C++ source):
        theta1 = shoulder yaw    = atan2(x, y)
        theta2 = femur pitch     = planar elbow angle (upper leg)
        theta3 = knee pitch      = planar elbow angle (lower leg)

    All angles are in DEGREES, matching the ESP servo output.
    """
    r = math.sqrt(x * x + y * y)

    # Law-of-cosines for the planar 2-link (b, c) after subtracting hip offset a
    D = ((r - a) * (r - a) + z * z - b * b - c * c) / (2.0 * b * c)

    if D > 1.0 or D < -1.0:
        return None   # target unreachable

    # theta3 — knee angle (elbow-up, D ≥ 0 → leg straight; D < 0 → bent)
    theta3 = math.atan2(math.sqrt(1.0 - D * D), D)

    # theta2 — femur angle
    theta2 = (
        math.atan2(-z, r - a)
        - math.atan2(c * math.sin(theta3), b + c * math.cos(theta3))
    )

    # theta1 — shoulder yaw (maps x/y lateral position to rotation)
    theta1 = math.atan2(x, y)

    # Convert to degrees — identical to the C++ RAD_TO_DEG multiplication
    theta1 *= _RAD_TO_DEG
    theta2 *= _RAD_TO_DEG
    theta3 *= _RAD_TO_DEG

    return theta1, theta2, theta3


# ---- inv_kin_global() ------------------------------------------------
def inv_kin_global(
    leg_end_pos: list[list[float]],   # shape [NUM_LEGS][3] — global foot positions
    xl: list[float],                  # leg origin X offset from body frame [mm]
    yl: list[float],                  # leg origin Y offset
    zl: list[float],                  # leg origin Z offset
    side_sign: list[int],             # +1 (right side) or -1 (left side) per leg
) -> list[tuple[float, float, float]] | None:
    """
    Port of my_bot_ik::inv_kin_global().

    Solves IK for all 4 legs simultaneously.

    Args:
        leg_end_pos : list of 4 [x, y, z] global foot target positions [mm].
        xl, yl, zl  : shoulder joint positions in the body frame [mm].
                      These are the (x,y,z) of each shoulder_N_joint origin
                      extracted from the URDF (converted from metres to mm).
        side_sign   : [+1, -1, +1, -1] for legs [1, 2, 3, 4].
                      Right-side legs use +1; left-side legs use -1.
                      Applied to y_local and theta1 to mirror the geometry.

    Returns:
        List of 4 (theta1_deg, theta2_deg, theta3_deg) tuples on success.
        None if any leg is unreachable.

    Shoulder origins from URDF (metres → mm):
        Leg 1 (front-right): shoulder_1_joint xyz = (300.69, 149.24, -115.84)
        Leg 2 (front-left) : shoulder_2_joint xyz = (300.69, 149.74, -307.54)
        Leg 3 (rear-right) : shoulder_3_joint xyz = ( 30.69, 149.24, -115.84)
        Leg 4 (rear-left)  : shoulder_4_joint xyz = ( 30.69, 149.24, -307.54)
    """
    results: list[tuple[float, float, float]] = []

    for i in range(NUM_LEGS):
        # Transform global foot position to local leg frame
        x_local = leg_end_pos[i][0] - xl[i]
        y_local = (leg_end_pos[i][1] - yl[i]) * side_sign[i]   # mirror left/right
        z_local = leg_end_pos[i][2] - zl[i]

        angles = inverse_kinematics(x_local, y_local, z_local)

        if angles is None:
            return None   # abort — this leg is unreachable

        theta1_deg, theta2_deg, theta3_deg = angles
        theta1_deg *= side_sign[i]   # mirror shoulder yaw for the other side

        results.append((theta1_deg, theta2_deg, theta3_deg))

    return results
