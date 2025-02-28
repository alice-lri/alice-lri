set(SRC_FILES
    FileUtils.cpp
)

set(HEADER_FILES
    FileUtils.h
)

add_executable(examples main.cpp ${SRC_FILES} ${HEADER_FILES})
add_executable(eigen_loop_vs_select eigen_loop_vs_select.cpp)
add_executable(eigen_lazy_vs_immediate eigen_lazy_vs_immediate.cpp)

target_link_libraries(examples PRIVATE accurate_ri)

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