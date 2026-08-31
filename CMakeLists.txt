cmake_minimum_required(VERSION 3.8)
project(tangent_bug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Tangent Bug navigation node
add_executable(tangent_bug_node src/tangent_bug_node.cpp)

ament_target_dependencies(tangent_bug_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

install(TARGETS
  tangent_bug_node
  DESTINATION lib/${PROJECT_NAME})

# Ship the RViz config so it can be found via:
#   rviz2 -d $(ros2 pkg prefix tangent_bug)/share/tangent_bug/rviz/tangent_bug.rviz
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # cpplint enforces the Google style guide, which this file does not follow
  # (clang-format layout, different naming). Skipped deliberately.
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
