cmake_minimum_required(VERSION 3.20)
project(alice_lri)
set(CMAKE_CXX_STANDARD 20)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}/lib")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}/examples")

add_subdirectory(lib ${CMAKE_BINARY_DIR}/lib)
add_subdirectory(examples ${CMAKE_BINARY_DIR}/examples)

enable_testing()

if(NOT LIB_MODE)
    add_subdirectory(tests ${CMAKE_BINARY_DIR}/tests)
endif ()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g" CACHE STRING "C++ flags for Debug builds" FORCE)
    set(CMAKE_C_FLAGS_DEBUG "-O0 -g" CACHE STRING "C flags for Debug builds" FORCE)
endif ()
