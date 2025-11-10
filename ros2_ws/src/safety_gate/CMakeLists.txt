cmake_minimum_required(VERSION 3.8)
project(safety_gate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(common_msgs REQUIRED)

# Include directories
include_directories(include)

# Safety gate node
add_executable(safety_gate_node src/safety_gate_node.cpp)
ament_target_dependencies(safety_gate_node
  rclcpp
  std_msgs
  geometry_msgs
  common_msgs
)

# Install targets
install(TARGETS
  safety_gate_node
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
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

