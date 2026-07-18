cmake_minimum_required(VERSION 3.5)
project(yumi_planning_interface)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  # 当前规划接口已经使用了 std::optional，
  # 因此编译标准至少需要提升到 C++17。
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(yumi_interfaces REQUIRED)

add_executable(yumi_planning_interface_node
  src/yumi_planning_interface.cpp
  src/yumi_planning_interface_node_main.cpp
)

ament_target_dependencies(yumi_planning_interface_node
  control_msgs
  rclcpp_action
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2
  tf2_eigen
  yumi_interfaces
)

add_executable(yumi_planning_scene_obstacles_node
  src/planning_scene_obstacles_node.cpp
)

ament_target_dependencies(yumi_planning_scene_obstacles_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  tf2
)

target_include_directories(yumi_planning_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  yumi_planning_interface_node
  yumi_planning_scene_obstacles_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
