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
find_package(osqp_vendor REQUIRED)

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

# Libraries
add_library(lie_group_controller_lib
  src/operational_space_control.cpp
  src/ik_newton_raphson.cpp
  src/ik_newton_raphson_qp_constraints.cpp
  src/euler_integration_gradient_descent.cpp
  src/modified_euler_integration_gradient_descent.cpp
  src/runge_kutta_gradient_descent.cpp
  src/exponential_coordinate_relative_jacobian_gradient_descent.cpp
  src/ik_newton_raphson_cmt.cpp
  src/ik_newton_raphson_body_jacobian.cpp
  src/ik_newton_raphson_body_jacobian_cmt.cpp
  src/ik_newton_raphson_passivity_cmt.cpp
  src/ik_newton_raphson_cmt_nric.cpp
  src/ik_newton_raphson_rac.cpp
  src/ik_newton_raphson_rac_nric.cpp
  src/arm_ik_newton_raphson_rac_nric.cpp
)
ament_target_dependencies(lie_group_controller_lib ${dependencies})
target_link_libraries(lie_group_controller_lib
  ${Eigen3_LIBRARIES}
  osqp::osqp
)

# Executables
add_executable(operational_space_control_node src/operational_space_control.cpp)
ament_target_dependencies(operational_space_control_node ${dependencies})
target_link_libraries(operational_space_control_node lie_group_controller_lib)

add_executable(ik_newton_raphson_node src/ik_newton_raphson.cpp)
ament_target_dependencies(ik_newton_raphson_node ${dependencies})
target_link_libraries(ik_newton_raphson_node lie_group_controller_lib)

add_executable(ik_newton_raphson_qp_constraints src/ik_newton_raphson_qp_constraints.cpp)
ament_target_dependencies(ik_newton_raphson_qp_constraints ${dependencies})
target_link_libraries(ik_newton_raphson_qp_constraints lie_group_controller_lib)

add_executable(euler_integration_gradient_descent src/euler_integration_gradient_descent.cpp)
ament_target_dependencies(euler_integration_gradient_descent ${dependencies})
target_link_libraries(euler_integration_gradient_descent lie_group_controller_lib)

add_executable(modified_euler_integration_gradient_descent src/modified_euler_integration_gradient_descent.cpp)
ament_target_dependencies(modified_euler_integration_gradient_descent ${dependencies})
target_link_libraries(modified_euler_integration_gradient_descent lie_group_controller_lib)

add_executable(runge_kutta_gradient_descent src/runge_kutta_gradient_descent.cpp)
ament_target_dependencies(runge_kutta_gradient_descent ${dependencies})
target_link_libraries(runge_kutta_gradient_descent lie_group_controller_lib)

add_executable(exponential_coordinate_relative_jacobian_gradient_descent src/exponential_coordinate_relative_jacobian_gradient_descent.cpp)
ament_target_dependencies(exponential_coordinate_relative_jacobian_gradient_descent ${dependencies})
target_link_libraries(exponential_coordinate_relative_jacobian_gradient_descent lie_group_controller_lib)

add_executable(ik_newton_raphson_cmt src/ik_newton_raphson_cmt.cpp)
ament_target_dependencies(ik_newton_raphson_cmt ${dependencies})
target_link_libraries(ik_newton_raphson_cmt lie_group_controller_lib)

add_executable(ik_newton_raphson_body_jacobian src/ik_newton_raphson_body_jacobian.cpp)
ament_target_dependencies(ik_newton_raphson_body_jacobian ${dependencies})
target_link_libraries(ik_newton_raphson_body_jacobian lie_group_controller_lib)

add_executable(ik_newton_raphson_body_jacobian_cmt src/ik_newton_raphson_body_jacobian_cmt.cpp)
ament_target_dependencies(ik_newton_raphson_body_jacobian_cmt ${dependencies})
target_link_libraries(ik_newton_raphson_body_jacobian_cmt lie_group_controller_lib)

add_executable(ik_newton_raphson_passivity_cmt src/ik_newton_raphson_passivity_cmt.cpp)
ament_target_dependencies(ik_newton_raphson_passivity_cmt ${dependencies})
target_link_libraries(ik_newton_raphson_passivity_cmt lie_group_controller_lib)

add_executable(ik_newton_raphson_cmt_nric src/ik_newton_raphson_cmt_nric.cpp)
ament_target_dependencies(ik_newton_raphson_cmt_nric ${dependencies})
target_link_libraries(ik_newton_raphson_cmt_nric lie_group_controller_lib)

add_executable(ik_newton_raphson_rac src/ik_newton_raphson_rac.cpp)
ament_target_dependencies(ik_newton_raphson_rac ${dependencies})
target_link_libraries(ik_newton_raphson_rac lie_group_controller_lib)

add_executable(ik_newton_raphson_rac_nric src/ik_newton_raphson_rac_nric.cpp)
ament_target_dependencies(ik_newton_raphson_rac_nric ${dependencies})
target_link_libraries(ik_newton_raphson_rac_nric lie_group_controller_lib)

add_executable(arm_ik_newton_raphson_rac_nric src/arm_ik_newton_raphson_rac_nric.cpp)
ament_target_dependencies(arm_ik_newton_raphson_rac_nric ${dependencies})
target_link_libraries(arm_ik_newton_raphson_rac_nric lie_group_controller_lib)

################################################################################
# Install
################################################################################
# Install libraries
install(TARGETS lie_group_controller_lib
  EXPORT export_lie_group_controller_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

# Install executables
install(TARGETS
  operational_space_control_node
  ik_newton_raphson_node
  ik_newton_raphson_qp_constraints
  euler_integration_gradient_descent
  modified_euler_integration_gradient_descent
  runge_kutta_gradient_descent
  exponential_coordinate_relative_jacobian_gradient_descent
  ik_newton_raphson_cmt
  ik_newton_raphson_body_jacobian
  ik_newton_raphson_body_jacobian_cmt
  ik_newton_raphson_passivity_cmt
  ik_newton_raphson_cmt_nric
  ik_newton_raphson_rac
  ik_newton_raphson_rac_nric
  arm_ik_newton_raphson_rac_nric
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
