cmake_minimum_required(VERSION 3.8)
project(controllers_lqr)

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

# LQR core library
add_library(lqr_core src/lqr_core.cpp)
ament_target_dependencies(lqr_core
  rclcpp
  std_msgs
  geometry_msgs
  common_msgs
)
target_link_libraries(lqr_core Eigen3::Eigen)

# LQR node
add_executable(lqr_node src/lqr_node.cpp)
ament_target_dependencies(lqr_node
  rclcpp
  std_msgs
  geometry_msgs
  common_msgs
)
target_link_libraries(lqr_node lqr_core Eigen3::Eigen)

# Install targets
install(TARGETS
  lqr_core
  lqr_node
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

  # LQR stability test
  ament_add_gtest(test_lqr_stability test/test_lqr_stability.cpp)
  target_include_directories(test_lqr_stability PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_lqr_stability lqr_core Eigen3::Eigen)
  ament_target_dependencies(test_lqr_stability
    rclcpp
    std_msgs
    geometry_msgs
    common_msgs
  )
endif()

ament_package()

