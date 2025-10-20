cmake_minimum_required(VERSION 3.20)
project(alice_lri_python_bindings LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Opt-in: Only the Python build turns this ON via pyproject.toml
option(PYBUILD_AUTO_CONAN "Run Conan for lib/ during Python build" OFF)
if (PYBUILD_AUTO_CONAN)
    execute_process(COMMAND conan profile show RESULT_VARIABLE _profile_show_res)

    if (NOT _profile_show_res EQUAL 0)
        message(STATUS "No Conan profile found. Running 'conan profile detect'.")
        execute_process(COMMAND conan profile detect RESULT_VARIABLE _profile_res)

        if (NOT _profile_res EQUAL 0)
            message(FATAL_ERROR "Conan profile detect failed (code ${_profile_res}).")
        endif ()
    endif ()

    execute_process(
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
            COMMAND conan install .
            -s compiler.cppstd=20
            -s build_type=Release
            -of "${CMAKE_BINARY_DIR}/lib"
            --build=missing
            RESULT_VARIABLE _conan_res
    )
    if (NOT _conan_res EQUAL 0)
        message(FATAL_ERROR "Conan install failed (code ${_conan_res}).")
    endif ()
endif ()

# Find pybind11
find_package(pybind11 REQUIRED)

# Add the core library
add_subdirectory(${CMAKE_SOURCE_DIR}/lib ${CMAKE_BINARY_DIR}/lib)

# Create Python module
pybind11_add_module(_alice_lri src/alice_lri/bindings.cpp)
target_link_libraries(_alice_lri PRIVATE alice_lri)
target_include_directories(_alice_lri PRIVATE ${CMAKE_SOURCE_DIR}/lib/include)

set_target_properties(_alice_lri PROPERTIES
        INSTALL_RPATH "$ORIGIN"
        BUILD_RPATH_USE_ORIGIN ON
)

install(TARGETS _alice_lri
        LIBRARY DESTINATION alice_lri
        COMPONENT python)

install(TARGETS alice_lri
        LIBRARY DESTINATION alice_lri COMPONENT python
        ARCHIVE DESTINATION alice_lri COMPONENT python
        RUNTIME DESTINATION alice_lri COMPONENT python
)