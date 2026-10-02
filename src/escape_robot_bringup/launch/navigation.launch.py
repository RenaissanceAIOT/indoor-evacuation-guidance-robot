from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup = get_package_share_directory("escape_robot_bringup")
    nav2 = get_package_share_directory("nav2_bringup")
    hardware = os.path.join(bringup, "launch", "hardware.launch.py")
    params = os.path.join(bringup, "config", "nav2_params.yaml")
    exits = os.path.join(bringup, "config", "exits.yaml")
    perception = os.path.join(bringup, "config", "perception.yaml")
    system = os.path.join(bringup, "config", "system.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    enable_perception = LaunchConfiguration("enable_perception")

    return LaunchDescription([
        DeclareLaunchArgument("map", description="Absolute path to the saved map YAML"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_perception", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "bringup_launch.py")),
            launch_arguments={
                "map": map_file,
                "params_file": params,
                "use_sim_time": use_sim_time,
                "autostart": "true",
            }.items(),
        ),
        Node(
            package="escape_robot_guidance",
            executable="evacuation_coordinator",
            parameters=[exits, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="escape_robot_guidance",
            executable="system_supervisor",
            parameters=[system, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="escape_robot_perception",
            executable="object_depth",
            parameters=[perception, {"use_sim_time": use_sim_time}],
            condition=IfCondition(enable_perception),
            output="screen",
        ),
    ])

