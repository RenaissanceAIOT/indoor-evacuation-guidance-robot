from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup = get_package_share_directory("escape_robot_bringup")
    hardware = os.path.join(bringup, "launch", "hardware.launch.py")
    slam_params = os.path.join(bringup, "config", "slam_toolbox.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])

