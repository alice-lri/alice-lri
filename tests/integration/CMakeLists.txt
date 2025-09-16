cmake_minimum_required(VERSION 3.20)
project(alice_lri_installation_test)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the installed alice_lri package
find_package(alice_lri REQUIRED)

# Create a simple test executable
add_executable(installation_test test_installation.cpp)

# Link against the alice_lri library
target_link_libraries(installation_test alice_lri::alice_lri)

# Enable testing
enable_testing()
add_test(NAME installation_test COMMAND installation_test)
