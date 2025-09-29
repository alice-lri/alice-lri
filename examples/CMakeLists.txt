set(SRC_FILES utils/FileUtils.cpp)
set(HEADER_FILES utils/FileUtils.h)

add_executable(some_tests some_tests.cpp ${SRC_FILES} ${HEADER_FILES})
add_executable(kitti_basic kitti_basic.cpp ${SRC_FILES} ${HEADER_FILES})

target_link_libraries(some_tests PRIVATE alice_lri)
target_link_libraries(kitti_basic PRIVATE alice_lri)
