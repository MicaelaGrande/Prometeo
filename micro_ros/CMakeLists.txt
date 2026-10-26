cmake_minimum_required(VERSION 3.16)

# Componente con todos los drivers de sensores (ver drivers/CMakeLists.txt)
set(EXTRA_COMPONENT_DIRS drivers)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(prometeo)
