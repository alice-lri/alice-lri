cmake_minimum_required(VERSION 3.22)
project(accurate_ri)
set(CMAKE_CXX_STANDARD 20)

execute_process(
        COMMAND conan install lib -s compiler.cppstd=gnu20 --output-folder=${CMAKE_BINARY_DIR}/lib --build=missing
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        RESULT_VARIABLE conan_result
)

# Check if Conan ran successfully
if (NOT conan_result EQUAL 0)
    message(FATAL_ERROR "Conan install failed!")
endif ()


add_subdirectory(lib)
add_subdirectory(examples)

enable_testing()
add_subdirectory(tests)

#option(ENABLE_PYTHON "Enable Python Bindings" ON)
#if(ENABLE_PYTHON)
#    add_subdirectory(python)
#endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g" CACHE STRING "C++ flags for Debug builds" FORCE)
    set(CMAKE_C_FLAGS_DEBUG "-O0 -g" CACHE STRING "C flags for Debug builds" FORCE)
endif ()
