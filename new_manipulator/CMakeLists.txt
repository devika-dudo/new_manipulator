cmake_minimum_required(VERSION 3.8)
project(new_manipulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── move_to_xyz executable ───────────────────────────────────────────────────
add_executable(move_to_xyz src/move_to_xyz.cpp)

ament_target_dependencies(move_to_xyz
  rclcpp
  geometry_msgs
  std_msgs
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
)

install(TARGETS move_to_xyz
  DESTINATION lib/${PROJECT_NAME}
)
# ────────────────────────────────────────────────────────────────────────────

install(
  DIRECTORY launch urdf config meshes worlds models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(gazebo_ros REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()