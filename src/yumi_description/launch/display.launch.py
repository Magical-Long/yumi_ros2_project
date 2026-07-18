import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_launch_setup(context):
    pkg_share = get_package_share_directory("yumi_description")
    default_model_path = os.path.join(pkg_share, "urdf", "yumi.urdf")

    model = LaunchConfiguration("model").perform(context)
    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    model_path = model if model else default_model_path

    with open(model_path, "r", encoding="utf-8") as model_file:
        robot_description_content = model_file.read()

    robot_description = {"robot_description": robot_description_content}

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_gui),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_gui),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return [
        joint_state_publisher_gui,
        joint_state_publisher,
        robot_state_publisher,
        rviz2,
    ]


def generate_launch_description():
    default_model_path = os.path.join(
        get_package_share_directory("yumi_description"),
        "urdf",
        "yumi.urdf",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model_path,
                description="Absolute path to the YuMi URDF file.",
            ),
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Whether to start joint_state_publisher_gui.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to start RViz2.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Whether to use /clock time.",
            ),
            OpaqueFunction(function=_build_launch_setup),
        ]
    )
