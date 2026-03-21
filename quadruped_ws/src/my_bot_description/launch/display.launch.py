"""
display.launch.py
-----------------
Launches robot_state_publisher + joint_state_publisher_gui + RViz2.
Use this to inspect the URDF without starting Gazebo.

  ros2 launch my_bot_description display.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_bot_description')

    urdf_path  = os.path.join(pkg, 'urdf', 'my_robot.urdf')
    rviz_path  = os.path.join(pkg, 'rviz', 'urdf.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # Broadcasts TF from URDF joint definitions
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # GUI sliders so you can manually move every joint
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            output='screen',
        ),
    ])
