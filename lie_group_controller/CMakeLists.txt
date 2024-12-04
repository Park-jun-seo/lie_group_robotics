################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(lie_group_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lie_toolbox REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(sensor_msgs REQUIRED)



################################################################################
# Build
################################################################################
include_directories(
  include
  ${yaml_cpp_INCLUDE_DIRS}
  ${Eigen3_LIBRARIES}
)

set(dependencies
  rclcpp
  rclcpp_action
  lie_toolbox
  Eigen3
  sensor_msgs
  geometry_msgs
)

add_library(lie_group_controller
  src/impedance_control_left.cpp
)

ament_target_dependencies(lie_group_controller ${dependencies})
target_link_libraries(lie_group_controller)

add_executable(impedance_control_left src/impedance_control_left.cpp)
ament_target_dependencies(impedance_control_left ${dependencies})


################################################################################
# Install
################################################################################
install(TARGETS lie_group_controller
  EXPORT export_lie_group_controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  impedance_control_left
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_package()
