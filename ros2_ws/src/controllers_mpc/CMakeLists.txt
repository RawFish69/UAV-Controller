cmake_minimum_required(VERSION 3.8)
project(controllers_mpc)

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

# MPC core library
add_library(mpc_core src/mpc_core.cpp)
ament_target_dependencies(mpc_core
  rclcpp
  std_msgs
  geometry_msgs
  common_msgs
)
target_link_libraries(mpc_core Eigen3::Eigen)

# MPC node
add_executable(mpc_node src/mpc_node.cpp)
ament_target_dependencies(mpc_node
  rclcpp
  std_msgs
  geometry_msgs
  common_msgs
)
target_link_libraries(mpc_node mpc_core Eigen3::Eigen)

# Install targets
install(TARGETS
  mpc_core
  mpc_node
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
endif()

ament_package()
