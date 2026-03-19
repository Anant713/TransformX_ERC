from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    description_pkg = FindPackageShare("my_bot_description")
    bringup_pkg = FindPackageShare("my_bot_bringup")
    ik_pkg = FindPackageShare("my_bot_ik")

    urdf_file = PathJoinSubstitution(
        [description_pkg, "urdf", "my_robot.urdf"]
    )

    controllers_file = PathJoinSubstitution(
        [bringup_pkg, "config", "controllers.yaml"]
    )

    ik_params_file = PathJoinSubstitution(
        [ik_pkg, "config", "ik_params.yaml"]
    )

    robot_description = {"robot_description": Command(["cat ", urdf_file])}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
        output="screen"
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "my_bot", "-topic", "robot_description"],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--param-file", controllers_file],
        output="screen"
    )

    ik_node = Node(
        package="my_bot_ik",
        executable="ik_node",
        output="screen",
        parameters=[ik_params_file]
    )

    foot_trajectory_node = Node(
        package="my_bot_ik",
        executable="foot_trajectory_node",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        ik_node,
        foot_trajectory_node
    ])