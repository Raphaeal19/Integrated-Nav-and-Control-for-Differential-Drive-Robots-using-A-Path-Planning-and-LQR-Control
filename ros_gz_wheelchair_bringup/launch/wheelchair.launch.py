import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node, SetParameter


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
            "obstacles_world.sdf"
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

    online_async_params = os.path.join(pkg_project_bringup, 'config', 'mapper_params_online_async.yaml')
    online_async_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_project_bringup, 'launch', 'online_async_launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'slam_params_file':online_async_params}.items()
    )

    navigation_params = os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml')
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_project_bringup, 'launch', 'navigation_launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'params_file': navigation_params}.items()
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        robot_state_publisher,
        DeclareLaunchArgument("rviz", default_value="true",
            description="Open RViz."),
        # control_node,
        bridge,
        gz_sim,
        rviz,
        TimerAction(
            period=3.0,
            actions=[online_async_launch,]
        ),
        TimerAction(
            period=3.0,
            actions=[navigation_launch,]
        )
    ]) 
