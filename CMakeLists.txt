cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS
        lib/
        lib/sensors
        lib/utilities/
        lib/drivers/components
        lib/homekit/components/homekit
)

add_compile_options(-fdiagnostics-color=always)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(nixie-clock C CXX)
