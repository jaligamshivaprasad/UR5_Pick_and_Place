cmake_minimum_required(VERSION 3.8)
project(ur5_adaptive_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(kdl_parser REQUIRED)

# Library for discrete PID core, gain adaptation, and payload compensation
add_library(pid_core
  src/pid_core.cpp
  src/gain_adaptation.cpp
  src/payload_compensator.cpp
  src/arm_dynamics_compensator.cpp
)
set_target_properties(pid_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(pid_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pid_core
  kdl_parser
)

# Controller plugin library
add_library(ur5_adaptive_control SHARED
  src/adaptive_arm_controller.cpp
)
target_include_directories(ur5_adaptive_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ur5_adaptive_control
  pid_core
  joint_trajectory_controller::joint_trajectory_controller
)
ament_target_dependencies(ur5_adaptive_control
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  joint_trajectory_controller
  control_toolbox
  realtime_tools
  control_msgs
  trajectory_msgs
  std_msgs
  pluginlib
  kdl_parser
)

pluginlib_export_plugin_description_file(controller_interface ur5_adaptive_control_plugins.xml)

install(TARGETS pid_core ur5_adaptive_control
  EXPORT export_ur5_adaptive_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES ur5_adaptive_control_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/controller_switcher.py
  scripts/gain_telemetry_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pid_core test/test_pid_core.cpp)
  target_link_libraries(test_pid_core pid_core)

  ament_add_gtest(test_gain_adaptation test/test_gain_adaptation.cpp)
  target_link_libraries(test_gain_adaptation pid_core)

  ament_add_gtest(test_payload_compensator test/test_payload_compensator.cpp)
  target_link_libraries(test_payload_compensator pid_core)
  ament_target_dependencies(test_payload_compensator kdl_parser)

  ament_add_gtest(test_phase9_safety test/test_phase9_safety.cpp)
  target_link_libraries(test_phase9_safety pid_core)
  ament_target_dependencies(test_phase9_safety kdl_parser)

  ament_add_gtest(test_arm_dynamics_compensator test/test_arm_dynamics_compensator.cpp)
  target_link_libraries(test_arm_dynamics_compensator pid_core)
  ament_target_dependencies(test_arm_dynamics_compensator kdl_parser)
endif()

ament_export_include_directories(include)
ament_export_libraries(pid_core ur5_adaptive_control)
ament_export_targets(export_ur5_adaptive_control)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  joint_trajectory_controller
  control_toolbox
  realtime_tools
  control_msgs
  trajectory_msgs
  std_msgs
  pluginlib
  kdl_parser
)

ament_package()

