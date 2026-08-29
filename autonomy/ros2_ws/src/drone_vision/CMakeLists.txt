cmake_minimum_required(VERSION 3.8)
project(drone_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(
  LIBCAMERA
  REQUIRED
  libcamera
)

add_executable(camera_sim_node src/camera_sim_node.cpp)
add_executable(vision_proc_node src/vision_proc_node.cpp)
add_executable(camera_node src/camera_node.cpp)
add_executable(camera_calibration src/camera_calibration_node.cpp)

ament_target_dependencies(
  camera_sim_node
  rclcpp
  sensor_msgs
  cv_bridge
)

ament_target_dependencies(
  vision_proc_node
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  tf2
)

ament_target_dependencies(
  camera_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(
  camera_calibration
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(
  camera_sim_node
  ${OpenCV_LIBS}
)

target_link_libraries(
  vision_proc_node
  ${OpenCV_LIBS}
)

target_link_libraries(
  camera_node
  ${LIBCAMERA_LIBRARIES}
)

target_link_libraries(
  camera_calibration
  ${OpenCV_LIBS}
)

target_include_directories(
  camera_node
  PRIVATE
  ${LIBCAMERA_INCLUDE_DIRS}
)

install(
  TARGETS camera_sim_node vision_proc_node camera_node camera_calibration
  DESTINATION lib/${PROJECT_NAME}
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
