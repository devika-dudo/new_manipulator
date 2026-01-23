cmake_minimum_required(VERSION 3.22)
project(new_manipulator_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(shape_msgs REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
# Install Python scripts
# Add C++ executable
add_executable(joystick_cartesian_teleop src/joystick_cartesian_teleop.cpp)
ament_target_dependencies(joystick_cartesian_teleop
  rclcpp
  rclcpp_action
  moveit_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  shape_msgs
)

# Install C++ executable
install(TARGETS
  joystick_cartesian_teleop
  DESTINATION lib/${PROJECT_NAME}
)
