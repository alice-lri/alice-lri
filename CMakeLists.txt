cmake_minimum_required(VERSION 3.20)
project(alice_lri)
set(CMAKE_CXX_STANDARD 20)

add_subdirectory(lib ${CMAKE_BINARY_DIR}/lib)
add_subdirectory(examples ${CMAKE_BINARY_DIR}/examples)

option(INCLUDE_TESTS "Include tests in the build" OFF)
if(INCLUDE_TESTS)
    enable_testing()
    add_subdirectory(tests ${CMAKE_BINARY_DIR}/tests)
endif ()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g" CACHE STRING "C++ flags for Debug builds" FORCE)
    set(CMAKE_C_FLAGS_DEBUG "-O0 -g" CACHE STRING "C flags for Debug builds" FORCE)
endif ()
