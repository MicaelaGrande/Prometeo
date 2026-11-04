cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config web
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/motor_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
