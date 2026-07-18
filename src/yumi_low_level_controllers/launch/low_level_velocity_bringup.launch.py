import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("yumi_low_level_controllers")
    default_config_path = os.path.join(package_share, "config", "low_level_velocity.yaml")

    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    low_level_controller_node = Node(
        package="yumi_low_level_controllers",
        executable="yumi_low_level_controller_node",
        name="yumi_low_level_controller",
        output="screen",
        parameters=[
            config_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_path,
            description="Parameter YAML file for the low-level controller.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Whether to use simulation time.",
        ),
        low_level_controller_node,
    ])
