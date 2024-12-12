cmake_minimum_required(VERSION 3.8)
project(final_project)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(final_project_msgs REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)

add_executable(openManX_forward_kinematics src/subscriber_forward_kinematics.cpp)
add_executable(openManX_inverse_kinematics src/service_inverse_kinematics.cpp)
add_executable(openManX_velocity_kinematics src/services_velocity_kinematics.cpp)
add_executable(openManX_velocity_control src/velocity_control.cpp)
add_executable(openManX_position_control src/position_control.cpp)

ament_target_dependencies(openManX_forward_kinematics rclcpp std_msgs sensor_msgs geometry_msgs Eigen3)
ament_target_dependencies(openManX_inverse_kinematics rclcpp std_msgs std_srvs geometry_msgs Eigen3 final_project_msgs)
ament_target_dependencies(openManX_velocity_kinematics rclcpp std_msgs std_srvs sensor_msgs geometry_msgs Eigen3 final_project_msgs)
ament_target_dependencies(openManX_velocity_control rclcpp std_msgs std_srvs Eigen3 final_project_msgs)
ament_target_dependencies(openManX_position_control rclcpp std_msgs dynamixel_sdk_custom_interfaces)

install(TARGETS
    openManX_forward_kinematics
    openManX_inverse_kinematics
    openManX_velocity_kinematics
    openManX_velocity_control
    openManX_position_control
        DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
