set(SRC_FILES
    FileUtils.cpp
)

set(HEADER_FILES
    FileUtils.h
)

include(${CMAKE_CURRENT_BINARY_DIR}/conan_toolchain.cmake)
find_package(SQLiteCpp REQUIRED)

find_package(nlohmann_json REQUIRED)
set(JSON_BuildTests OFF CACHE INTERNAL "")

find_package(Eigen3 REQUIRED)
include_directories(Eigen3_INCLUDE_DIRS)

add_executable(some_tests some_tests.cpp ${SRC_FILES} ${HEADER_FILES})
add_executable(intrinsics_to_sql intrinsics_to_sql.cpp ${SRC_FILES} ${HEADER_FILES})
add_executable(measure_times measure_times.cpp ${SRC_FILES} ${HEADER_FILES})
add_executable(kitti_basic kitti_basic.cpp ${SRC_FILES} ${HEADER_FILES})

target_link_libraries(some_tests PRIVATE alice_lri)
target_link_libraries(intrinsics_to_sql PRIVATE alice_lri SQLiteCpp nlohmann_json::nlohmann_json)
target_link_libraries(measure_times PRIVATE alice_lri)
target_link_libraries(kitti_basic PRIVATE alice_lri)

target_include_directories(some_tests PRIVATE ${Eigen3_INCLUDE_DIRS})
target_include_directories(intrinsics_to_sql PRIVATE ${Eigen3_INCLUDE_DIRS})
target_include_directories(measure_times PRIVATE ${Eigen3_INCLUDE_DIRS})
target_include_directories(kitti_basic PRIVATE ${Eigen3_INCLUDE_DIRS})
