from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Planning + RViz demo without ros2_control.

    Use when ros2_control_node fails to start (e.g. diagnostic_updater ABI mismatch).
    Supports Plan in RViz; Execute requires the full demo.launch.py with controllers.
    """
    moveit_config = MoveItConfigsBuilder(
        "lite_000_asm", package_name="lite_moveit2"
    ).to_moveit_configs()
    pkg = moveit_config.package_path

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg / "launch/static_virtual_joint_tfs.launch.py")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg / "launch/rsp.launch.py")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg / "launch/move_group.launch.py")),
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", str(pkg / "config/moveit.rviz")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
