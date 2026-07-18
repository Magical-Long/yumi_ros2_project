cmake_minimum_required(VERSION 3.5)
project(yumi_interfaces)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # 基础位姿/参考/状态消息。
  "msg/Pose6D.msg"
  "msg/TaskSpacePose.msg"
  "msg/CooperativeTaskCommand.msg"
  # cooperative_controller -> task_manager 的启动握手和规划状态反馈。
  "msg/CooperativeControllerStatus.msg"
  "msg/JointSpaceReference.msg"
  "msg/LowLevelControllerStatus.msg"
  "msg/TaskSpaceReference.msg"
  # 任务空间轨迹点和整条轨迹，用于 cooperative_controller 缓存播放。
  "msg/TaskSpaceTrajectoryPoint.msg"
  "msg/TaskSpaceTrajectory.msg"
  # planning_interface 对外暴露的 MoveIt 包装 action。
  "action/PlanArmPose.action"
  "action/PlanBothArmsPoses.action"
  "action/PlanTaskSpaceTrajectory.action"
  "action/ResetToHome.action"
  "action/ResetBothArmsToHome.action"
  "action/RunRandomMotion.action"
  DEPENDENCIES builtin_interfaces trajectory_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
