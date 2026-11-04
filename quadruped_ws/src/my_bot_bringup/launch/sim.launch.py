"""
sim.launch.py
-------------
Master launch file. Starts everything needed for IK testing in Gazebo Sim:

  1. Gazebo Sim  (via my_bot_gazebo/spawn.launch.py)
       └─ robot_state_publisher
       └─ spawn robot into Gazebo
       └─ controller spawners (joint_state_broadcaster,
                               leg_position_controller,
                               wheel_velocity_controller)

  2. IK Solver node  (my_bot_control/ik_solver_node.py)

Usage:
  ros2 launch my_bot_bringup sim.launch.py

After launch, send a foot target to move leg 1:
  ros2 topic pub --once /foot_target geometry_msgs/PointStamped \
    '{header: {frame_id: "leg_1"}, point: {x: 0.0, y: 80.0, z: -200.0}}'

Topic reference:
  /foot_target                                    ← your input
  /leg_position_controller/joint_trajectory       ← IK → controller
  /joint_states                                   ← controller → everyone
  /tf  /tf_static                                 ← robot_state_publisher
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo  = get_package_share_directory('my_bot_gazebo')
    pkg_control = get_package_share_directory('my_bot_control')

    # ------------------------------------------------------------------ #
    # 1.  Gazebo + RSP + spawn + controllers                               #
    # ------------------------------------------------------------------ #
    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'spawn.launch.py')
        )
    )

    # ------------------------------------------------------------------ #
    # 2. IK Solver node                                                    #
    #    Delayed by 5 s to ensure controllers are active before the node  #
    #    starts accepting targets.                                         #
    # ------------------------------------------------------------------ #
    ik_node = Node(
        package='my_bot_control',
        executable='ik_solver_node',
        name='ik_solver',
        output='screen',
        parameters=[
            os.path.join(pkg_control, 'config', 'robot_params.yaml'),
        ],
    )

    ik_node_delayed = TimerAction(period=5.0, actions=[ik_node])

    return LaunchDescription([
        gazebo_bringup,
        ik_node_delayed,
    ])
