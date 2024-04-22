import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_wheelchair_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_wheelchair_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_wheelchair_description')
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Load the SDF file from the "description" package
    sdf_file = os.path.join(pkg_project_description, "models", "wheelchair", "chair_sdf.sdf")
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim,"launch","gz_sim.launch.py")),
        launch_arguments={"gz_args": PathJoinSubstitution([
            pkg_project_gazebo,
            "worlds",
            "world_corridor.sdf"
            ])
            }.items(),
        )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        name = "robot_state_publisher",
        output = "both",
        parameters = [
            {"use_sim_time": True},
            {"robot_description": robot_desc},
            ]
        )

    # Visualize in Rviz
    rviz = Node(
        package = "rviz2",
        executable = "rviz2",
        arguments = ["-d", os.path.join(pkg_project_bringup, "config", "wheelchair.rviz")],
        condition = IfCondition(LaunchConfiguration("rviz"))
        )

    # Bridge Ros topics and Gazebo messages for establishing communication
    bridge = Node(
        package = "ros_gz_bridge",
        executable = "parameter_bridge",
        parameters = [{
            "config_file": os.path.join(pkg_project_bringup, "config", "wheelchair_bridge.yaml"),
            "qos_overrides./tf_static.publisher.durability": "transient_local"
            } ],
        output = "screen"
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros_gz_wheelchair_bringup"),
            "config",
            "my_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    delay_rviz_after_jsbs = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[rviz]
        )
    )

    delay_controller_after_jsbs = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner]
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        DeclareLaunchArgument("rviz", default_value="true",
            description="Open RViz."),
        # control_node,
        bridge,
        gz_sim,
        rviz,
        # delay_rviz_after_jsbs,
        # delay_controller_after_jsbs,
        ])
