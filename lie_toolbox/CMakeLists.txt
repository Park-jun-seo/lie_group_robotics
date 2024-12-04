cmake_minimum_required(VERSION 3.12)
project(lie_toolbox)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Werror)
endif()

if($ENV{ROS_DISTRO} STREQUAL "foxy")
  add_definitions(-DROS_DISTRO=foxy)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
  add_definitions(-DROS_DISTRO=humble)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include/
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  geometry_msgs
  Eigen3
  rosidl_generator_cpp
  rclcpp_action
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

# if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
#   add_library(lie_toolbox
#     src/qos_profiles.cpp
#   )
# else()
#   add_library(lie_toolbox
#     src/lie_algebra.cpp
#     src/incremental_jacobian.cpp
#   )
# endif()

add_library(lie_toolbox
src/lie_algebra.cpp
src/incremental_jacobian.cpp
)

ament_target_dependencies(lie_toolbox ${dependencies})

if(NOT ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  target_link_libraries(lie_toolbox yaml-cpp ${Eigen3_LIBRARIES})
else()
  target_link_libraries(lie_toolbox yaml-cpp)
endif()

install(TARGETS lie_toolbox
  EXPORT export_lie_toolbox
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/
)
install(DIRECTORY include/
  DESTINATION include/
)

ament_export_targets(export_lie_toolbox HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# ###############################################################################
# Test
# ###############################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ###############################################################################
# Macro for ament package
# ###############################################################################
ament_package()
