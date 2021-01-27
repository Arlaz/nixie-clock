cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS
        lib/
)

add_compile_options(-fdiagnostics-color=always)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(nixie-clock C CXX)
