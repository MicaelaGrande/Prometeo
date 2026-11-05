cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config web
  DESTINATION share/${PROJECT_NAME}
)

# Nodos de Python: simulacion del HC-SR04 y test de motores. Van a lib/, que es donde
# buscan los launch
install(
  PROGRAMS scripts/hcsr04_sim.py scripts/modelo_hcsr04.py scripts/motor_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
