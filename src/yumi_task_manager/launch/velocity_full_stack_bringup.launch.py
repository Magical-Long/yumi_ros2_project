import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    require_trajectory_action_servers = LaunchConfiguration(
        "require_trajectory_action_servers"
    )
    enable_planning_scene_obstacles = LaunchConfiguration("enable_planning_scene_obstacles")

    move_group_delay = LaunchConfiguration("move_group_delay")
    planning_interface_delay = LaunchConfiguration("planning_interface_delay")
    initial_ready_reference_delay = LaunchConfiguration("initial_ready_reference_delay")
    task_manager_delay = LaunchConfiguration("task_manager_delay")
    cooperative_controller_delay = LaunchConfiguration("cooperative_controller_delay")
    low_level_controller_delay = LaunchConfiguration("low_level_controller_delay")

    task_manager_config = LaunchConfiguration("task_manager_config")
    cooperative_controller_config = LaunchConfiguration("cooperative_controller_config")
    low_level_controller_config = LaunchConfiguration("low_level_controller_config")

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yumi_simulation_interfaces"),
                "launch",
                "sim_bringup_velocity.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gui": gui,
        }.items(),
    )

    # MoveIt 的 move_group 只负责规划服务与 PlanningScene，不直接执行当前 velocity 链路。
    # 因此它可以晚于 Gazebo 启动，但必须早于 planning_interface 的 MoveGroupInterface 初始化。
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yumi_moveit_config"),
                "launch",
                "move_group_only.launch.py",
            )
        )
    )

    # planning_interface 是 MoveIt action 封装层。
    # cooperative_controller 后续会通过 PlanBothArmsPoses action 向它请求关节轨迹。
    planning_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yumi_planning_interface"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "require_trajectory_action_servers": require_trajectory_action_servers,
            "enable_planning_scene_obstacles": enable_planning_scene_obstacles,
        }.items(),
    )

    # task_manager 是最高层状态/序列发布者。
    # 它启动得最晚，并且还会等待 cooperative_controller_status.ready_for_task 握手。
    task_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yumi_task_manager"),
                "launch",
                "task_manager_bringup.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "config_file": task_manager_config,
        }.items(),
    )

    # cooperative_controller 连接 task_manager 与 planning/lowlevel：
    # 自由空间请求 MoveIt 轨迹，约束空间生成 task-space reference。
    cooperative_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yumi_cooperative_controller"),
                "launch",
                "cooperative_velocity_bringup.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "config_file": cooperative_controller_config,
        }.items(),
    )

    # low_level_controller 需要尽早启动，先接收临时 ready joint reference，
    # 让 Gazebo 里的双臂从初始状态收敛到安全 ready 姿态。
    low_level_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yumi_low_level_controllers"),
                "launch",
                "low_level_velocity_bringup.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "config_file": low_level_controller_config,
        }.items(),
    )

    # 临时 ready publisher 是启动阶段的“安全收拢”动作。
    # 它不经过 task_manager/cooperative_controller，直接给低层一个 joint-space reference。
    initial_ready_reference_publisher = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--rate",
            "20",
            "--times",
            "160",
            "/joint_space_reference",
            "yumi_interfaces/msg/JointSpaceReference",
            (
                "{reference_mode: 2, "
                "left_joint_names: ["
                "yumi_joint_1_l, yumi_joint_2_l, yumi_joint_7_l, yumi_joint_3_l, "
                "yumi_joint_4_l, yumi_joint_5_l, yumi_joint_6_l], "
                "right_joint_names: ["
                "yumi_joint_1_r, yumi_joint_2_r, yumi_joint_7_r, yumi_joint_3_r, "
                "yumi_joint_4_r, yumi_joint_5_r, yumi_joint_6_r], "
                "left_positions: [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0], "
                "right_positions: [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0], "
                "left_velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "
                "right_velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "
                "left_efforts: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "
                "right_efforts: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Whether to use the Gazebo clock across the whole stack.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Whether to start the Gazebo GUI window.",
            ),
            DeclareLaunchArgument(
                "require_trajectory_action_servers",
                default_value="false",
                description="Whether planning_interface should wait for FollowJointTrajectory action servers.",
            ),
            DeclareLaunchArgument(
                "enable_planning_scene_obstacles",
                default_value="true",
                description="Whether planning_interface should add configured PlanningScene obstacles.",
            ),
            DeclareLaunchArgument(
                "move_group_delay",
                default_value="25.0",
                description="Seconds to wait after Gazebo before starting move_group_only.",
            ),
            DeclareLaunchArgument(
                "planning_interface_delay",
                default_value="35.0",
                description="Seconds to wait after Gazebo before starting planning_interface.",
            ),
            DeclareLaunchArgument(
                "initial_ready_reference_delay",
                default_value="13.0",
                description="Seconds to wait before publishing a temporary ready joint reference.",
            ),
            DeclareLaunchArgument(
                "task_manager_delay",
                default_value="45.0",
                description="Seconds to wait after Gazebo before starting task_manager.",
            ),
            DeclareLaunchArgument(
                "cooperative_controller_delay",
                default_value="40.0",
                description="Seconds to wait after Gazebo before starting cooperative_controller.",
            ),
            DeclareLaunchArgument(
                "low_level_controller_delay",
                default_value="10.0",
                description="Seconds to wait after Gazebo before starting low_level_controller.",
            ),
            DeclareLaunchArgument(
                "task_manager_config",
                default_value=os.path.join(
                    get_package_share_directory("yumi_task_manager"),
                    "config",
                    "task_manager_skeleton.yaml",
                ),
                description="Parameter YAML for task_manager.",
            ),
            DeclareLaunchArgument(
                "cooperative_controller_config",
                default_value=os.path.join(
                    get_package_share_directory("yumi_cooperative_controller"),
                    "config",
                    "cooperative_velocity.yaml",
                ),
                description="Parameter YAML for cooperative_controller.",
            ),
            DeclareLaunchArgument(
                "low_level_controller_config",
                default_value=os.path.join(
                    get_package_share_directory("yumi_low_level_controllers"),
                    "config",
                    "low_level_velocity.yaml",
                ),
                description="Parameter YAML for low_level_controller.",
            ),
            simulation_launch,
            TimerAction(period=low_level_controller_delay, actions=[low_level_controller_launch]),
            TimerAction(
                period=initial_ready_reference_delay,
                actions=[initial_ready_reference_publisher],
            ),
            TimerAction(period=move_group_delay, actions=[move_group_launch]),
            TimerAction(period=planning_interface_delay, actions=[planning_interface_launch]),
            TimerAction(
                period=cooperative_controller_delay,
                actions=[cooperative_controller_launch],
            ),
            TimerAction(period=task_manager_delay, actions=[task_manager_launch]),
        ]
    )
