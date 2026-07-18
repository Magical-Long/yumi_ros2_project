cmake_minimum_required(VERSION 3.5)
project(yumi_task_manager)

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
find_package(rclcpp REQUIRED)
find_package(yumi_interfaces REQUIRED)

add_executable(yumi_task_manager_node
  src/yumi_task_manager_node.cpp
)

ament_target_dependencies(yumi_task_manager_node
  rclcpp
  yumi_interfaces
)

install(TARGETS yumi_task_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/publish_task_space_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
