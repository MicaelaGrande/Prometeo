cmake_minimum_required(VERSION 3.8)
project(prometeo_gazebo)

find_package(ament_cmake REQUIRED)

# Copia estas carpetas a install/ al compilar. Ahi es donde ROS las busca,
# no en src/. Por eso, despues de cambiar un archivo hay que compilar de nuevo.
install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
