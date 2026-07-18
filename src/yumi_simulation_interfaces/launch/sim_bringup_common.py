import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _remove_block(content, start_marker, end_marker):
    start = content.find(start_marker)
    if start == -1:
        return content

    end = content.find(end_marker, start)
    if end == -1:
        return content

    end += len(end_marker)
    if end < len(content) and content[end] == "\n":
        end += 1
    return content[:start] + content[end:]


def generate_sim_bringup_description(
    urdf_filename,
    controllers_yaml_filename,
    left_arm_controller_name,
    right_arm_controller_name,
    auto_home_supported,
):
    def _build_launch_setup(context):
        pkg_share = get_package_share_directory("yumi_simulation_interfaces")
        gazebo_ros_share = get_package_share_directory("gazebo_ros")

        use_sim_time = LaunchConfiguration("use_sim_time")
        gui = LaunchConfiguration("gui")
        pause = LaunchConfiguration("pause")
        world = LaunchConfiguration("world")
        enable_ros2_control = LaunchConfiguration("enable_ros2_control")
        auto_home = LaunchConfiguration("auto_home")
        home_delay = LaunchConfiguration("home_delay")
        entity_name = LaunchConfiguration("entity_name")
        controller_manager_name = LaunchConfiguration("controller_manager_name")
        x = LaunchConfiguration("x")
        y = LaunchConfiguration("y")
        z = LaunchConfiguration("z")

        urdf_path = os.path.join(pkg_share, "urdf", urdf_filename)
        controllers_yaml_path = os.path.join(
            pkg_share, "config", controllers_yaml_filename
        )
        robot_param_node = "/robot_state_publisher"

        with open(urdf_path, "r", encoding="utf-8") as urdf_file:
            robot_description_content = urdf_file.read()

        if enable_ros2_control.perform(context).lower() not in ("true", "1", "yes", "on"):
            robot_description_content = _remove_block(
                robot_description_content,
                '  <ros2_control name="yumi_gazebo_system" type="system">',
                "  </ros2_control>",
            )
            robot_description_content = _remove_block(
                robot_description_content,
                '  <gazebo>\n    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">',
                "  </gazebo>",
            )

        robot_description_content = robot_description_content.replace(
            "__CONTROLLERS_YAML_PATH__",
            controllers_yaml_path,
        )
        robot_description_content = robot_description_content.replace(
            "__ROBOT_PARAM_NODE__",
            robot_param_node,
        )

        robot_description = {"robot_description": robot_description_content}

        generated_urdf = tempfile.NamedTemporaryFile(
            mode="w",
            suffix=".urdf",
            prefix="yumi_gazebo_ros2_",
            delete=False,
            dir="/tmp",
            encoding="utf-8",
        )
        generated_urdf.write(robot_description_content)
        generated_urdf.flush()
        generated_urdf.close()

        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
            ),
            launch_arguments={
                "gui": gui,
                "pause": pause,
                "world": world,
            }.items(),
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

        spawn_robot = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_yumi",
            output="screen",
            arguments=[
                "-entity",
                entity_name,
                "-file",
                generated_urdf.name,
                "-x",
                x,
                "-y",
                y,
                "-z",
                z,
            ],
        )

        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner.py",
            name="joint_state_broadcaster_spawner",
            output="screen",
            condition=IfCondition(enable_ros2_control),
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                controller_manager_name,
                "--controller-manager-timeout",
                "120",
            ],
        )

        left_arm_controller_spawner = Node(
            package="controller_manager",
            executable="spawner.py",
            name="left_arm_controller_spawner",
            output="screen",
            condition=IfCondition(enable_ros2_control),
            arguments=[
                left_arm_controller_name,
                "--controller-manager",
                controller_manager_name,
                "--controller-manager-timeout",
                "120",
            ],
        )

        right_arm_controller_spawner = Node(
            package="controller_manager",
            executable="spawner.py",
            name="right_arm_controller_spawner",
            output="screen",
            condition=IfCondition(enable_ros2_control),
            arguments=[
                right_arm_controller_name,
                "--controller-manager",
                controller_manager_name,
                "--controller-manager-timeout",
                "120",
            ],
        )

        left_home_command = ExecuteProcess(
            cmd=[
                "ros2",
                "action",
                "send_goal",
                "/left_arm_controller/follow_joint_trajectory",
                "control_msgs/action/FollowJointTrajectory",
                (
                    "trajectory:\n"
                    "  joint_names:\n"
                    "    - yumi_joint_1_l\n"
                    "    - yumi_joint_2_l\n"
                    "    - yumi_joint_7_l\n"
                    "    - yumi_joint_3_l\n"
                    "    - yumi_joint_4_l\n"
                    "    - yumi_joint_5_l\n"
                    "    - yumi_joint_6_l\n"
                    "  points:\n"
                    "    - positions: [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
                    "      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
                    "      time_from_start: {sec: 4, nanosec: 0}\n"
                ),
            ],
            output="screen",
            condition=IfCondition(auto_home),
        )

        right_home_command = ExecuteProcess(
            cmd=[
                "ros2",
                "action",
                "send_goal",
                "/right_arm_controller/follow_joint_trajectory",
                "control_msgs/action/FollowJointTrajectory",
                (
                    "trajectory:\n"
                    "  joint_names:\n"
                    "    - yumi_joint_1_r\n"
                    "    - yumi_joint_2_r\n"
                    "    - yumi_joint_7_r\n"
                    "    - yumi_joint_3_r\n"
                    "    - yumi_joint_4_r\n"
                    "    - yumi_joint_5_r\n"
                    "    - yumi_joint_6_r\n"
                    "  points:\n"
                    "    - positions: [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
                    "      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
                    "      time_from_start: {sec: 4, nanosec: 0}\n"
                ),
            ],
            output="screen",
            condition=IfCondition(auto_home),
        )

        load_joint_state_broadcaster = RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    TimerAction(
                        period=3.0,
                        actions=[joint_state_broadcaster_spawner],
                    )
                ],
            )
        )

        load_controllers = RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[left_arm_controller_spawner, right_arm_controller_spawner],
            )
        )

        actions = [
            gazebo,
            robot_state_publisher,
            spawn_robot,
            load_joint_state_broadcaster,
            load_controllers,
        ]

        if auto_home_supported:
            load_auto_home = RegisterEventHandler(
                OnProcessExit(
                    target_action=right_arm_controller_spawner,
                    on_exit=[
                        TimerAction(
                            period=home_delay,
                            actions=[left_home_command, right_home_command],
                            condition=IfCondition(auto_home),
                        )
                    ],
                )
            )
            actions.append(load_auto_home)

        return actions

    default_world_path = os.path.join(
        get_package_share_directory("yumi_simulation_interfaces"),
        "world",
        "final.world",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Whether to use simulated clock from Gazebo.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Whether to start the Gazebo client GUI.",
            ),
            DeclareLaunchArgument(
                "pause",
                default_value="false",
                description="Whether to start Gazebo in paused mode.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=default_world_path,
                description="Gazebo world file path.",
            ),
            DeclareLaunchArgument(
                "enable_ros2_control",
                default_value="true",
                description="Whether to enable gazebo_ros2_control and spawn controllers.",
            ),
            DeclareLaunchArgument(
                "auto_home",
                default_value="true" if auto_home_supported else "false",
                description="Whether to send both arms to the collision-free home pose after controllers become active.",
            ),
            DeclareLaunchArgument(
                "home_delay",
                default_value="2.0",
                description="Seconds to wait after arm controller activation before sending the home trajectory.",
            ),
            DeclareLaunchArgument(
                "robot_description_topic",
                default_value="/robot_description",
                description="Topic that spawn_entity.py reads to get the robot XML description.",
            ),
            DeclareLaunchArgument(
                "entity_name",
                default_value="yumi",
                description="Entity name used when spawning the robot in Gazebo.",
            ),
            DeclareLaunchArgument(
                "controller_manager_name",
                default_value="/controller_manager",
                description="Fully-qualified controller_manager node name used by controller spawners.",
            ),
            DeclareLaunchArgument(
                "x",
                default_value="0.0",
                description="Initial x position of the robot in the Gazebo world frame.",
            ),
            DeclareLaunchArgument(
                "y",
                default_value="0.0",
                description="Initial y position of the robot in the Gazebo world frame.",
            ),
            DeclareLaunchArgument(
                "z",
                default_value="0.0",
                description="Initial z position of the robot in the Gazebo world frame.",
            ),
            OpaqueFunction(function=_build_launch_setup),
        ]
    )
