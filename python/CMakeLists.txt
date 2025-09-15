cmake_minimum_required(VERSION 3.20)
project(alice_lri_python_bindings LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find pybind11
find_package(pybind11 REQUIRED)

# Add the core library
add_subdirectory(${CMAKE_SOURCE_DIR}/../lib ${CMAKE_BINARY_DIR}/lib)

# Create Python module
pybind11_add_module(_alice_lri src/alice_lri/bindings.cpp)
target_link_libraries(_alice_lri PRIVATE alice_lri)
target_include_directories(_alice_lri PRIVATE ${CMAKE_SOURCE_DIR}/../lib/include)

set_target_properties(_alice_lri PROPERTIES
        BUILD_RPATH "$ORIGIN/../lib"
        INSTALL_RPATH "$ORIGIN/../lib"
        BUILD_RPATH_USE_ORIGIN ON
)

# Install the Python package manually if needed
install(TARGETS _alice_lri LIBRARY DESTINATION alice_lri RUNTIME DESTINATION alice_lri COMPONENT python)