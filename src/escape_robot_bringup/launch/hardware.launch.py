from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup = get_package_share_directory("escape_robot_bringup")
    description = get_package_share_directory("escape_robot_description")
    config = os.path.join(bringup, "config", "base.yaml")
    ekf_config = os.path.join(bringup, "config", "ekf.yaml")
    xacro_file = os.path.join(description, "urdf", "escape_robot.urdf.xacro")
    port = LaunchConfiguration("port")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/escape_base"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[{"source_list": ["/arm/joint_states"], "use_sim_time": use_sim_time}],
        ),
        Node(
            package="escape_robot_base",
            executable="base_driver",
            parameters=[config, {"port": port, "use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[ekf_config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
