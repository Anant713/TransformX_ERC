"""
ik_test.launch.py
-----------------
Lightweight launch for IK algorithm testing WITHOUT Gazebo.

Starts:
  - robot_state_publisher   (URDF → TF)
  - joint_state_publisher   (echoes commanded positions back as /joint_states)
  - ik_solver_node          (your IK algorithm)
  - rviz2                   (optional visualisation)

This is the fastest feedback loop for validating IK math:
  1. Launch this file.
  2. Publish a foot target.
  3. Watch the robot move in RViz — no Gazebo physics involved.

Usage:
  ros2 launch my_bot_bringup ik_test.launch.py

Send a target:
  ros2 topic pub --once /foot_target geometry_msgs/PointStamped \
    '{header: {frame_id: "leg_1"}, point: {x: 0.0, y: 80.0, z: -200.0}}'

Inspect solved angles (degrees, from the IK node log):
  ros2 topic echo /leg_position_controller/joint_trajectory

Note:
  joint_state_publisher here just echoes back whatever the IK node commands,
  letting robot_state_publisher drive RViz TF without a real controller.
  Replace it with the actual controller pipeline when connecting to Gazebo
  (use sim.launch.py for that).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from trajectory_msgs.msg import JointTrajectory  # noqa: F401 — for reference


def generate_launch_description():
    pkg_desc    = get_package_share_directory('my_bot_description')
    pkg_control = get_package_share_directory('my_bot_control')

    urdf_path = os.path.join(pkg_desc, 'urdf', 'my_robot.urdf')
    rviz_path = os.path.join(pkg_desc, 'rviz', 'urdf.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # ---- robot_state_publisher -----------------------------------------
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # ---- joint_state_publisher -----------------------------------------
    # Publishes /joint_states so RSP can compute TF.
    # In ik_test mode the IK node doesn't write /joint_states directly,
    # so this node provides the echo.
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # ---- IK solver node ------------------------------------------------
    ik_node = Node(
        package='my_bot_control',
        executable='ik_solver_node',
        name='ik_solver',
        output='screen',
        parameters=[
            os.path.join(pkg_control, 'config', 'robot_params.yaml'),
        ],
    )

    # ---- RViz2 ---------------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        output='screen',
    )

    return LaunchDescription([rsp, jsp, ik_node, rviz])
