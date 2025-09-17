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

target_link_libraries(alice_lri PRIVATE)

add_executable(examples main.cpp ${SRC_FILES} ${HEADER_FILES})
add_executable(examples_sql main_sql.cpp ${SRC_FILES} ${HEADER_FILES})
add_executable(measure_times measure_times.cpp ${SRC_FILES} ${HEADER_FILES})
add_executable(eigen_loop_vs_select eigen_loop_vs_select.cpp)
add_executable(eigen_lazy_vs_immediate eigen_lazy_vs_immediate.cpp)

target_link_libraries(examples PRIVATE alice_lri)
target_link_libraries(examples_sql PRIVATE alice_lri SQLiteCpp nlohmann_json::nlohmann_json)
target_link_libraries(measure_times PRIVATE alice_lri)

target_include_directories(examples PRIVATE ${Eigen3_INCLUDE_DIRS})
target_include_directories(examples_sql PRIVATE ${Eigen3_INCLUDE_DIRS})
target_include_directories(measure_times PRIVATE ${Eigen3_INCLUDE_DIRS})
target_include_directories(eigen_loop_vs_select PRIVATE ${Eigen3_INCLUDE_DIRS})
target_include_directories(eigen_lazy_vs_immediate PRIVATE ${Eigen3_INCLUDE_DIRS})