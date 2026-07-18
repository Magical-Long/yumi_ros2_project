from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    auto_home = LaunchConfiguration("auto_home")
    home_delay = LaunchConfiguration("home_delay")
    moveit_delay = LaunchConfiguration("moveit_delay")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yumi_simulation_interfaces"),
                "launch",
                "sim_bringup_traj.launch.py",
            )
        ),
        launch_arguments={
            "auto_home": auto_home,
            "home_delay": home_delay,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yumi_moveit_config"),
                "launch",
                "demo.launch.py",
            )
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "use_sim_time": "true",
            "start_state_publisher": "false",
        }.items(),
    )

    delayed_moveit_launch = TimerAction(
        period=moveit_delay,
        actions=[moveit_launch],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to start RViz2 alongside MoveIt.",
            ),
            DeclareLaunchArgument(
                "auto_home",
                default_value="true",
                description="Whether to let the simulation layer send both arms to the collision-free home pose before starting MoveIt.",
            ),
            DeclareLaunchArgument(
                "home_delay",
                default_value="2.0",
                description="Seconds to wait before the simulation layer sends the home trajectory.",
            ),
            DeclareLaunchArgument(
                "moveit_delay",
                default_value="11.0",
                description="Seconds to wait before starting MoveIt after simulation launch.",
            ),
            sim_launch,
            delayed_moveit_launch,
        ]
    )
