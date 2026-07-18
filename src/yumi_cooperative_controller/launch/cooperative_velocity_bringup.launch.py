import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("yumi_cooperative_controller")
    default_config_path = os.path.join(package_share, "config", "cooperative_velocity.yaml")

    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    cooperative_controller_node = Node(
        package="yumi_cooperative_controller",
        executable="yumi_cooperative_controller_node",
        name="yumi_cooperative_controller",
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
            description="Parameter YAML file for the cooperative controller stack.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Whether to use the simulated Gazebo clock.",
        ),
        cooperative_controller_node,
    ])
