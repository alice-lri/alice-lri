set(SRC_FILES
    FileUtils.cpp
)

set(HEADER_FILES
    FileUtils.h
)

include(${CMAKE_BINARY_DIR}/examples/conan_toolchain.cmake)
find_package(SQLiteCpp REQUIRED)

find_package(nlohmann_json REQUIRED)
set(JSON_BuildTests OFF CACHE INTERNAL "")
target_link_libraries(accurate_ri PRIVATE
        nlohmann_json::nlohmann_json
)

add_executable(examples main.cpp ${SRC_FILES} ${HEADER_FILES})
add_executable(examples_sql main_sql.cpp ${SRC_FILES} ${HEADER_FILES})
add_executable(eigen_loop_vs_select eigen_loop_vs_select.cpp)
add_executable(eigen_lazy_vs_immediate eigen_lazy_vs_immediate.cpp)

target_link_libraries(examples PRIVATE accurate_ri)
target_link_libraries(examples_sql PRIVATE accurate_ri SQLiteCpp nlohmann_json::nlohmann_json)

if(ENABLE_STRICT_WARNINGS) #TODO make this trigger in release mode
    message(STATUS "Strict warnings enabled")
    target_compile_options(examples PRIVATE
            -Wall -Wextra -Wpedantic  # General best practices
            -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align
            -Woverloaded-virtual -Wconversion -Wsign-conversion
            -Wpessimizing-move -Wredundant-move -Werror=unused-value
            -Werror=unused-variable -Werror=unused-parameter
    )
endif()