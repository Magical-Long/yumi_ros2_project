import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("yumi_task_manager")
    default_config_path = os.path.join(package_share, "config", "task_manager_skeleton.yaml")

    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    task_manager_node = Node(
        package="yumi_task_manager",
        executable="yumi_task_manager_node",
        name="yumi_task_manager",
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
            description="Task manager skeleton parameter file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Whether to use simulated time.",
        ),
        task_manager_node,
    ])
