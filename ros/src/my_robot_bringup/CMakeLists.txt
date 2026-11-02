cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Nodos de Python (simulacion del HC-SR04): van a lib/, que es donde buscan los launch
install(PROGRAMS scripts/hcsr04_sim.py scripts/modelo_hcsr04.py scripts/arm_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
