"""
spawn.launch.py
---------------
1. Starts Gazebo Sim with flat_ground.sdf
2. Publishes robot_description via robot_state_publisher
3. Spawns the URDF model into Gazebo
4. Activates all ros2_control controllers

Usage (standalone):
  ros2 launch my_bot_gazebo spawn.launch.py

Or included from my_bot_bringup/launch/sim.launch.py.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_desc   = get_package_share_directory('my_bot_description')
    pkg_gazebo = get_package_share_directory('my_bot_gazebo')

    urdf_path  = os.path.join(pkg_desc, 'urdf', 'my_robot.urdf')
    world_path = os.path.join(pkg_gazebo, 'worlds', 'flat_ground.sdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # ------------------------------------------------------------------ #
    # 1. Gazebo Sim                                                        #
    # ------------------------------------------------------------------ #
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # ------------------------------------------------------------------ #
    # 2. robot_state_publisher                                             #
    # ------------------------------------------------------------------ #
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ------------------------------------------------------------------ #
    # 3. Spawn robot into Gazebo                                           #
    # ------------------------------------------------------------------ #
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name', 'my_bot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.25',   # spawn slightly above ground
        ],
    )

    # ------------------------------------------------------------------ #
    # 4. ros2_control controllers — start after spawn completes           #
    # ------------------------------------------------------------------ #
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    leg_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='leg_position_controller_spawner',
        arguments=['leg_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    wheel_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='wheel_velocity_controller_spawner',
        arguments=['wheel_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Start controllers only after the robot has been spawned.
    # OnProcessExit triggers when 'spawn' exits (spawn_entity exits after success).
    start_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn,
            on_exit=[
                joint_state_broadcaster,
                leg_controller,
                wheel_controller,
            ],
        )
    )

    return LaunchDescription([
        gz_sim,
        rsp,
        spawn,
        start_controllers,
    ])
