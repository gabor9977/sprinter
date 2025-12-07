import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('sprinter_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'sprinter.xacro')
    controllers_file = os.path.join(pkg_path, 'config', 'sprinter_controllers.yaml')
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'controllers_file': controllers_file}
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    bolt_motion_node = Node(
        package="sprinter_control",
        executable="bolt_motion",
        parameters=[{"hip_z": 0.90, "repeats": 5}],
        output="screen",
    )
 
    sprint_controller_node = Node(
        package="sprinter_control",
        executable="sprint_controller",
        output="screen",
    )
    
    # gait_node = Node(
    #     package="sprinter_control",
    #     executable="gait_jointstate",
    #     parameters=[{"repeats": 5, "cycle_duration": 6.31}],
    #     output="screen",
    # )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-entity', 'sprinter'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        bolt_motion_node,
        sprint_controller_node,
        # gait_node,  # Deactivated - executable doesn't exist
    ])