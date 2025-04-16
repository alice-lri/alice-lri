find_package(GTest REQUIRED)

# Add test executable
add_executable(accurate_ri_tests 
    tests.cpp
    accurate_ri_tests.cpp
    hough_transform_tests.cpp
    stats_tests.cpp
    vertical_intrinsics_tests.cpp
    point_array_tests.cpp
        utils_tests.cpp
)

# Find the Eigen package
find_package(Eigen3 REQUIRED)

# Include directories
target_include_directories(accurate_ri_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/src
    ${CMAKE_SOURCE_DIR}/lib/include
    ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory
)

# Link with the library and Google Test
target_link_libraries(accurate_ri_tests
    accurate_ri
    gtest
    gtest_main
    Eigen3::Eigen  # Link with Eigen if necessary
)

# Add the test to CTest
add_test(NAME accurate_ri_tests COMMAND accurate_ri_tests)
