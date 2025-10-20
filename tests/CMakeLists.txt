include(${CMAKE_CURRENT_BINARY_DIR}/conan_toolchain.cmake)
find_package(GTest REQUIRED)

include(${CMAKE_SOURCE_DIR}/lib/sources.cmake)
add_executable(alice_lri_tests ${ALICE_LRI_SOURCES}
        tests.cpp
        alice_array_tests.cpp
        alice_lri_tests.cpp
        hough_transform_tests.cpp
        stats_tests.cpp
        point_array_tests.cpp
        utils_tests.cpp
)
target_compile_definitions(alice_lri_tests PRIVATE ALICE_LRI_WHITE_BOX=1)

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED)

target_include_directories(alice_lri_tests PRIVATE
        ${CMAKE_BINARY_DIR}/lib/src
        ${CMAKE_SOURCE_DIR}/lib/src
        ${CMAKE_SOURCE_DIR}/lib/include
        ${Eigen3_INCLUDE_DIRS}
        ${nlohmann_json_INCLUDE_DIRS}
)

target_link_libraries(alice_lri_tests gtest::gtest Eigen3::Eigen boost::boost)

include(GoogleTest)
gtest_discover_tests(alice_lri_tests)