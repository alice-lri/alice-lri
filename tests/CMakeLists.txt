include(${CMAKE_CURRENT_BINARY_DIR}/conan_toolchain.cmake)
find_package(GTest REQUIRED)

# Add test executable
add_executable(alice_lri_tests 
    tests.cpp
    alice_lri_tests.cpp
    hough_transform_tests.cpp
    stats_tests.cpp
    point_array_tests.cpp
        utils_tests.cpp
)

# Find the Eigen package
find_package(Eigen3 REQUIRED)

# Include directories
target_include_directories(alice_lri_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/src
    ${CMAKE_SOURCE_DIR}/lib/include
    ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory
)

# Link with the library and Google Test
target_link_libraries(alice_lri_tests
    alice_lri
    gtest
    gtest_main
    Eigen3::Eigen  # Link with Eigen if necessary
)

# Add the test to CTest
include(GoogleTest)
gtest_discover_tests(alice_lri_tests)