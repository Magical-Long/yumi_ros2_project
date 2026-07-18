cmake_minimum_required(VERSION 3.5)
project(yumi_low_level_controllers)

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
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yumi_interfaces REQUIRED)

add_executable(yumi_low_level_controller_node
  src/yumi_low_level_controller.cpp
  src/yumi_low_level_controller_node_main.cpp
)

ament_target_dependencies(yumi_low_level_controller_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  std_msgs
  yumi_interfaces
)

target_include_directories(yumi_low_level_controller_node SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

target_link_libraries(yumi_low_level_controller_node
  ${pinocchio_LIBRARIES}
)

target_include_directories(yumi_low_level_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  yumi_low_level_controller_node
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
