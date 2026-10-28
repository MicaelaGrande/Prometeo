cmake_minimum_required(VERSION 3.16)

# drivers: componente con todos los drivers (ver drivers/CMakeLists.txt)
# esp_components: cliente micro-ROS instalado fuera del repo
set(
  EXTRA_COMPONENT_DIRS
    drivers
    "$ENV{HOME}/esp_components"
)

if(DEFINED ENV{MICROROS_NAMESPACE})
    add_compile_definitions(MICROROS_NAMESPACE="$ENV{MICROROS_NAMESPACE}")
else()
    add_compile_definitions(MICROROS_NAMESPACE="")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(prometeo)

