cmake_minimum_required(VERSION 3.20)
project(accurate_ri_python_bindings LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find pybind11
find_package(pybind11 REQUIRED)

# Add the core library
add_subdirectory(${CMAKE_SOURCE_DIR}/../lib ${CMAKE_BINARY_DIR}/lib)

# Create Python module
pybind11_add_module(_accurate_ri src/accurate_ri/bindings.cpp)
target_link_libraries(_accurate_ri PRIVATE accurate_ri)
target_include_directories(_accurate_ri PRIVATE ${CMAKE_SOURCE_DIR}/../lib/include)

set_target_properties(_accurate_ri PROPERTIES
        BUILD_RPATH "$ORIGIN/../lib"
        INSTALL_RPATH "$ORIGIN/../lib"
        BUILD_RPATH_USE_ORIGIN ON
)

# Install the Python package manually if needed
install(TARGETS _accurate_ri LIBRARY DESTINATION accurate_ri RUNTIME DESTINATION accurate_ri COMPONENT python)