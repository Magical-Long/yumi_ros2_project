import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_file(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return file.read()


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    run_demo_on_startup = LaunchConfiguration("run_demo_on_startup")
    execute_demo_plan_on_startup = LaunchConfiguration("execute_demo_plan_on_startup")
    reset_to_home_after_demo = LaunchConfiguration("reset_to_home_after_demo")
    demo_mode = LaunchConfiguration("demo_mode")
    require_trajectory_action_servers = LaunchConfiguration("require_trajectory_action_servers")
    enable_planning_scene_obstacles = LaunchConfiguration("enable_planning_scene_obstacles")
    log_level = LaunchConfiguration("log_level")

    # 规划接口节点和 move_group 必须共享同一份 robot_description，
    # 否则 MoveGroupInterface 无法正确构造机器人模型。
    robot_description = {
        "robot_description": load_file("yumi_description", "urdf/yumi.urdf")
    }
    # MoveIt 的规划组、命名姿态和允许碰撞关系都来自 SRDF。
    # 如果缺少这份语义描述，arm_l / arm_r 这样的规划组名称就不会存在。
    robot_description_semantic = {
        "robot_description_semantic": load_file(
            "yumi_moveit_config", "config/yumi.srdf"
        )
    }
    # 逆解与规划约束参数也和 move_group 保持一致，避免接口层和配置层各用各的参数。
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "yumi_moveit_config", "config/kinematics.yaml"
        )
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "yumi_moveit_config", "config/joint_limits.yaml"
        )
    }
    task_space_trajectory_config = load_yaml(
        "yumi_planning_interface", "config/task_space_trajectory.yaml"
    )
    planning_scene_obstacles_config = os.path.join(
        get_package_share_directory("yumi_planning_interface"),
        "config",
        "planning_scene_obstacles.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Whether the planning interface should use the Gazebo clock.",
            ),
            DeclareLaunchArgument(
                "run_demo_on_startup",
                default_value="false",
                description="Whether to run the one-shot random pose planning demo after startup.",
            ),
            DeclareLaunchArgument(
                "execute_demo_plan_on_startup",
                default_value="false",
                description="Whether the startup demo should execute the generated plan after planning succeeds.",
            ),
            DeclareLaunchArgument(
                "reset_to_home_after_demo",
                default_value="true",
                description="Whether the startup demo should reset the robot back to the named home pose after finishing.",
            ),
            DeclareLaunchArgument(
                "demo_mode",
                default_value="dual_arm",
                description="Startup demo mode: 'single_arm' or 'dual_arm'.",
            ),
            DeclareLaunchArgument(
                "require_trajectory_action_servers",
                default_value="false",
                description="Whether FollowJointTrajectory action servers must exist at startup.",
            ),
            DeclareLaunchArgument(
                "enable_planning_scene_obstacles",
                default_value="true",
                description="Whether to add configured box obstacles into MoveIt PlanningScene.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for the planning interface node.",
            ),
            Node(
                package="yumi_planning_interface",
                executable="yumi_planning_interface_node",
                name="yumi_planning_interface",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    task_space_trajectory_config,
                    {
                        "use_sim_time": use_sim_time,
                        "run_demo_on_startup": run_demo_on_startup,
                        "execute_demo_plan_on_startup": execute_demo_plan_on_startup,
                        "reset_to_home_after_demo": reset_to_home_after_demo,
                        "demo_mode": demo_mode,
                        "require_trajectory_action_servers": require_trajectory_action_servers,
                    },
                ],
            ),
            Node(
                package="yumi_planning_interface",
                executable="yumi_planning_scene_obstacles_node",
                name="yumi_planning_scene_obstacles",
                output="screen",
                condition=IfCondition(enable_planning_scene_obstacles),
                parameters=[
                    planning_scene_obstacles_config,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
