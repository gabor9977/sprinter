from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro  # <-- IMPORTANT


def generate_launch_description():
    # Resolve and process the Xacro to a raw XML string
    pkg_path = get_package_share_directory("sprinter_description")
    xacro_file = os.path.join(pkg_path, "urdf", "sprinter_simple.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    rviz_cfg = os.path.join(pkg_path, "urdf", "sprinter.rviz")

    # Parameters
    repeats = 1
    cycle_duration = 6.31  # seconds for 0→60 m
    hip_z = 0.90

    # 1) Robot State Publisher with the processed XML string
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # 2) RViz (camera follows base_link)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
    )

    # 3) Finish line markers (MarkerArray in /finish_line)
    markers = Node(
        package="sprinter_control",
        executable="finish_line_markers",
        output="screen",
    )

    # 4) Leg animation (JointState) — runs for exactly repeats * cycle_duration
    gait = Node(
        package="sprinter_control",
        executable="gait_jointstate",
        parameters=[{"repeats": repeats, "cycle_duration": cycle_duration}],
        output="screen",
    )

    # 5) TF world→base_link motion: 0→60m, reset to 0, repeat, then stop
    motion = Node(
        package="sprinter_control",
        executable="bolt_motion", 
        parameters=[{"hip_z": hip_z, "repeats": repeats}],
        output="screen",
    )

    # Start markers immediately after RViz starts; start gait+motion a bit later
    start_after_rviz = RegisterEventHandler(
        OnProcessStart(
            target_action=rviz,
            on_start=[
                TimerAction(period=0.5, actions=[markers]),
                TimerAction(period=6.0, actions=[gait, motion]),
            ],
        )
    )

    return LaunchDescription([rsp, rviz, start_after_rviz])
