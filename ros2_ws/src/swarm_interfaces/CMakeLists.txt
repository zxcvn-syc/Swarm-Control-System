cmake_minimum_required(VERSION 3.16)
project(swarm_interfaces LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# Find all interface packages we depend on BEFORE rosidl_generate_interfaces().
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(SWARM_INTERFACES_MSG_FILES
  "msg/TargetTrack.msg"
  "msg/TargetTrackArray.msg"
  "msg/TaskAssignment.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SWARM_INTERFACES_MSG_FILES}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()