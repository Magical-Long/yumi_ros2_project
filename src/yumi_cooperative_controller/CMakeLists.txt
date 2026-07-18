cmake_minimum_required(VERSION 3.5)
project(yumi_cooperative_controller)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# geometry_msgs 提供 WrenchStamped，用于订阅 Gazebo 末端力/力矩传感器。
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
# tf2/tf2_ros 用于把左右末端局部 wrench 的 force 旋转到 world frame。
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yumi_interfaces REQUIRED)

add_executable(yumi_cooperative_controller_node
  src/yumi_cooperative_controller.cpp
  src/yumi_cooperative_controller_node_main.cpp
)

ament_target_dependencies(yumi_cooperative_controller_node
  rclcpp
  rclcpp_action
  # wrench 订阅、TF 坐标统一，以及自定义 task/reference/status 消息都在 cooperative 层使用。
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  trajectory_msgs
  yumi_interfaces
)

target_include_directories(yumi_cooperative_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  yumi_cooperative_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
