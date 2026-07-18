import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 这个入口只负责启动 MoveIt 运行时本身，
    # 不启动 RViz，也不重复启动 robot_state_publisher。
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("yumi_moveit_config"),
                        "launch",
                        "demo.launch.py",
                    )
                ),
                launch_arguments={
                    "use_rviz": "false",
                    "start_state_publisher": "false",
                    "use_sim_time": "true",
                }.items(),
            )
        ]
    )
