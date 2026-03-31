cmake_minimum_required(VERSION 3.8)
project(corsmal_hrh_2026)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACAKGE_DEPENDENCIES
std_srvs
geometry_msgs
motion_planning_abstractions
motion_planning_abstractions_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion_planning_abstractions REQUIRED)
find_package(motion_planning_abstractions_msgs REQUIRED)


add_executable(human_robot_handover
  src/human_robot_handover.cpp
)
ament_target_dependencies(human_robot_handover ${THIS_PACAKGE_DEPENDENCIES})
install(
  TARGETS human_robot_handover
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
