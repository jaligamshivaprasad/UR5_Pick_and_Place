cmake_minimum_required(VERSION 3.8)
project(ur5_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(pick_place_node src/pick_place_node.cpp)
ament_target_dependencies(pick_place_node
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  control_msgs
  std_msgs
  tf2_msgs
  shape_msgs
)
install(TARGETS pick_place_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch scripts worlds
  DESTINATION share/${PROJECT_NAME}
)
install(FILES EFFORT_MODE.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scene_publisher_node.py
  scripts/robotiq_mimic_publisher.py
  scripts/pick_place_trajectory_orchestrator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_robotiq_mimic_publisher
    test/test_robotiq_mimic_publisher.py
  )
endif()

ament_package()
