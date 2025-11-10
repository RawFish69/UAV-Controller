cmake_minimum_required(VERSION 3.8)
project(controllers_pid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# PID core library
add_library(pid_core src/pid_core.cpp)
ament_target_dependencies(pid_core
  rclcpp
  std_msgs
  geometry_msgs
  common_msgs
)
target_link_libraries(pid_core Eigen3::Eigen)

# PID node
add_executable(pid_node src/pid_node.cpp)
ament_target_dependencies(pid_node
  rclcpp
  std_msgs
  geometry_msgs
  common_msgs
)
target_link_libraries(pid_node pid_core Eigen3::Eigen)

# Install targets
install(TARGETS
  pid_core
  pid_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()

  # PID anti-windup test
  ament_add_gtest(test_pid_antiwindup test/test_pid_antiwindup.cpp)
  target_include_directories(test_pid_antiwindup PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_pid_antiwindup pid_core Eigen3::Eigen)
  ament_target_dependencies(test_pid_antiwindup
    rclcpp
    std_msgs
    geometry_msgs
    common_msgs
  )
endif()

ament_package()

