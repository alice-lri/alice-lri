cmake_minimum_required(VERSION 3.22)
project(accurate_ri)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(lib)
add_subdirectory(examples)

enable_testing()
add_subdirectory(tests)

option(ENABLE_PYTHON "Enable Python Bindings" ON)
if(ENABLE_PYTHON)
    add_subdirectory(python)
endif()
