cmake_minimum_required(VERSION 3.16)

# Capas (cada una solo usa las de abajo):
#   main/    micro-ROS: topics, callbacks y arranque
#   logica/  sensores, freno de emergencia, centrado (ver logica/CMakeLists.txt)
#   drivers/ hardware: una libreria por carpeta (motores+encoders, hcsr04, mg90s), cada una
#            con su CMakeLists, su Kconfig y un proyecto ejemplo/ con su propio sdkconfig
# esp_components: cliente micro-ROS instalado fuera del repo
set(
  EXTRA_COMPONENT_DIRS
    drivers/motores
    drivers/hcsr04
    drivers/mg90s
    logica
    "$ENV{HOME}/esp_components"
)

if(DEFINED ENV{MICROROS_NAMESPACE})
    add_compile_definitions(MICROROS_NAMESPACE="$ENV{MICROROS_NAMESPACE}")
else()
    add_compile_definitions(MICROROS_NAMESPACE="")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(prometeo)
