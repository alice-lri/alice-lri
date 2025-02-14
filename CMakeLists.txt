cmake_minimum_required(VERSION 3.22)
project(accurate_ri)

set(CMAKE_CXX_STANDARD 20)
list(APPEND CMAKE_PREFIX_PATH "/home/samuel.soutullo/.miniconda3/envs/accurate_ri_env")

add_subdirectory(lib)
add_subdirectory(examples)

enable_testing()
add_subdirectory(tests)

option(ENABLE_PYTHON "Enable Python Bindings" ON)
if(ENABLE_PYTHON)
    add_subdirectory(python)
endif()
