cmake_minimum_required(VERSION 3.8)
project(drone_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(telemetry_node src/telemetry_node.cpp)
add_executable(command_node src/command_node.cpp)
add_executable(bridge_sim_node src/bridge_sim_node.cpp)
add_executable(guidance_node src/guidance_node.cpp)

ament_target_dependencies(
  telemetry_node
  rclcpp
  std_msgs
)

ament_target_dependencies(
  command_node
  rclcpp
  geometry_msgs
)

ament_target_dependencies(
  bridge_sim_node
  rclcpp
  geometry_msgs
  std_msgs
)

ament_target_dependencies(
  guidance_node
  rclcpp
  geometry_msgs
)

install(
  TARGETS 
    telemetry_node 
    command_node 
    bridge_sim_node 
    guidance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
