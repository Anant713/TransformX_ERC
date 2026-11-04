#!/usr/bin/env python3
"""
ik_solver_node.py
-----------------
ROS 2 node that bridges foot-space targets to joint-space commands.

Subscribes:
    /foot_target  (geometry_msgs/PointStamped)
        frame_id field encodes which leg: "leg_1", "leg_2", "leg_3", "leg_4"
        point.x/y/z are the desired foot position in the body frame [mm].

Publishes:
    /leg_position_controller/joint_trajectory  (trajectory_msgs/JointTrajectory)
        Directly consumed by the JointTrajectoryController loaded by ros2_control.

Service:
    /solve_ik  (std_srvs/Trigger equivalent — see below)
        Not used in this minimal version; reserved for later.

No custom message types are needed — standard geometry_msgs and
trajectory_msgs are sufficient for IK testing.

Joint name → IK angle mapping (from URDF ros2_control section):
    shoulder_N_joint    ← theta1 (yaw,  shoulder)
    femur_top_1_joint   ← theta2 (pitch, femur)    [leg 1 only — URDF naming quirk]
    femur_N_joint       ← theta2                    [legs 2-4]
    leg_N_joint         ← theta3 (pitch, knee)

All IK angles are returned in degrees by kinematics.py and converted
to radians here before publishing (Gazebo controllers expect radians).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ---- Joint name table ------------------------------------------------
# Order must match ros2_control.yaml → leg_position_controller → joints
# Indexed as:  LEG_JOINTS[leg_index] = (shoulder, femur, leg_link)
LEG_JOINTS = {
    'leg_1': ('shoulder_1_joint', 'femur_top_1_joint', 'leg_1_joint'),
    'leg_2': ('shoulder_2_joint', 'femur_2_joint',     'leg_2_joint'),
    'leg_3': ('shoulder_3_joint', 'femur_3_joint',     'leg_3_joint'),
    'leg_4': ('shoulder_4_joint', 'femur_4_joint',     'leg_4_joint'),
}

# All 12 joints in the exact order declared in ros2_control.yaml
ALL_LEG_JOINTS = [
    'shoulder_1_joint', 'femur_top_1_joint', 'leg_1_joint',
    'shoulder_2_joint', 'femur_2_joint',     'leg_2_joint',
    'shoulder_3_joint', 'femur_3_joint',     'leg_3_joint',
    'shoulder_4_joint', 'femur_4_joint',     'leg_4_joint',
]


class IKSolverNode(Node):

    def __init__(self):
        super().__init__('ik_solver')

        # ---- Parameters ----------------------------------------------
        self.declare_parameter('execution_time_sec', 0.5)
        self.exec_time = self.get_parameter('execution_time_sec').value

        # ---- Lazy-import kinematics (no ROS dependency) --------------
        from my_bot_control.kinematics import inverse_kinematics
        self._ik = inverse_kinematics

        # ---- Cached joint state: current angles per joint (radians) --
        # Initialised to zero; updated as /foot_target messages arrive.
        self._joint_state: dict[str, float] = {j: 0.0 for j in ALL_LEG_JOINTS}

        # ---- Publisher -----------------------------------------------
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            '/leg_position_controller/joint_trajectory',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

        # ---- Subscriber ----------------------------------------------
        self._foot_sub = self.create_subscription(
            PointStamped,
            '/foot_target',
            self._foot_target_cb,
            10,
        )

        self.get_logger().info(
            'IK Solver node ready.\n'
            '  Subscribe: /foot_target  (geometry_msgs/PointStamped)\n'
            '    frame_id = "leg_1" | "leg_2" | "leg_3" | "leg_4"\n'
            '    point.x/y/z = foot position in body frame [mm]\n'
            '  Publish:   /leg_position_controller/joint_trajectory'
        )

    # ------------------------------------------------------------------
    def _foot_target_cb(self, msg: PointStamped) -> None:
        leg_id = msg.header.frame_id   # e.g. "leg_1"

        if leg_id not in LEG_JOINTS:
            self.get_logger().warn(
                f'Unknown leg_id in frame_id: "{leg_id}". '
                f'Expected one of {list(LEG_JOINTS.keys())}'
            )
            return

        x, y, z = msg.point.x, msg.point.y, msg.point.z

        # ---- Solve IK ------------------------------------------------
        result = self._ik(x, y, z)

        if result is None:
            self.get_logger().warn(
                f'[{leg_id}] IK failed — target ({x:.1f}, {y:.1f}, {z:.1f}) mm '
                f'is out of reach (a={60}, b={95}, c={180} mm).'
            )
            return

        theta1_deg, theta2_deg, theta3_deg = result

        self.get_logger().debug(
            f'[{leg_id}] IK solved: '
            f'θ1={theta1_deg:.2f}° θ2={theta2_deg:.2f}° θ3={theta3_deg:.2f}°'
        )

        # ---- Convert degrees → radians for the joint controller ------
        shoulder_j, femur_j, leg_j = LEG_JOINTS[leg_id]
        self._joint_state[shoulder_j] = math.radians(theta1_deg)
        self._joint_state[femur_j]    = math.radians(theta2_deg)
        self._joint_state[leg_j]      = math.radians(theta3_deg)

        # ---- Build and publish JointTrajectory -----------------------
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names  = ALL_LEG_JOINTS

        point = JointTrajectoryPoint()
        point.positions = [self._joint_state[j] for j in ALL_LEG_JOINTS]
        point.time_from_start = Duration(
            sec=int(self.exec_time),
            nanosec=int((self.exec_time % 1) * 1e9),
        )
        traj.points = [point]

        self._traj_pub.publish(traj)

        self.get_logger().info(
            f'[{leg_id}] Published joint trajectory '
            f'(θ1={theta1_deg:.1f}° θ2={theta2_deg:.1f}° θ3={theta3_deg:.1f}°)'
        )


# ---- Entry point -----------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
