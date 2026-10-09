cmake_minimum_required(VERSION 3.8)
project(ur5_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch scripts worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scene_publisher_node.py
  scripts/robotiq_mimic_publisher.py
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
