cmake_minimum_required(VERSION 3.8)
project(prometeo_gazebo)

find_package(ament_cmake REQUIRED)

# Copia estas carpetas a install/ al compilar. Ahi es donde ROS las busca,
# no en src/. Por eso, despues de cambiar un archivo hay que compilar de nuevo.
install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

# Los nodos de Python van a lib/, que es donde ros2 run y los launch buscan programas
install(PROGRAMS scripts/hcsr04_sim.py scripts/modelo_hcsr04.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
