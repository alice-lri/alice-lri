cmake_minimum_required(VERSION 3.20)
project(alice_lri_python_bindings LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Opt-in: Only the Python build turns this ON via pyproject.toml
option(PYBUILD_AUTO_CONAN "Run Conan for lib/ during Python build" OFF)

if(PYBUILD_AUTO_CONAN)
    execute_process(
            COMMAND conan install "${CMAKE_SOURCE_DIR}/lib"
            -s compiler.cppstd=gnu20
            -s build_type=Release
            -of "${CMAKE_BINARY_DIR}/lib"
            --build=missing
            RESULT_VARIABLE _conan_res
    )
    if(NOT _conan_res EQUAL 0)
        message(FATAL_ERROR "Conan install failed (code ${_conan_res}).")
    endif()
endif()

# Find pybind11
find_package(pybind11 REQUIRED)

# Ensure the inner library installs into the Python package structure
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/python_install" CACHE PATH "" FORCE)

# Force its library dir to be under alice_lri/lib inside the wheel
set(CMAKE_INSTALL_LIBDIR "alice_lri/lib" CACHE PATH "Library install dir" FORCE)
set(CMAKE_INSTALL_BINDIR "alice_lri/lib" CACHE PATH "Binary install dir" FORCE)

# Add the core library
add_subdirectory(${CMAKE_SOURCE_DIR}/lib ${CMAKE_BINARY_DIR}/lib)

# Create Python module
pybind11_add_module(_alice_lri src/alice_lri/bindings.cpp)
target_link_libraries(_alice_lri PRIVATE alice_lri)
target_include_directories(_alice_lri PRIVATE ${CMAKE_SOURCE_DIR}/lib/include)

set_target_properties(_alice_lri PROPERTIES
        BUILD_RPATH "$ORIGIN/lib"
        INSTALL_RPATH "$ORIGIN/lib"
        BUILD_RPATH_USE_ORIGIN ON
)

install(TARGETS _alice_lri
        LIBRARY DESTINATION alice_lri
        COMPONENT python)

install(TARGETS alice_lri
        LIBRARY DESTINATION alice_lri/lib COMPONENT python
        ARCHIVE DESTINATION alice_lri/lib COMPONENT python
        RUNTIME DESTINATION alice_lri/lib COMPONENT python
)