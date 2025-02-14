add_executable(examples main.cpp)

target_link_libraries(examples PRIVATE accurate_ri)

if(ENABLE_STRICT_WARNINGS) #TODO make this trigger in release mode
    target_compile_options(accurate_ri PRIVATE
            -Wall -Wextra -Wpedantic  # General best practices
            -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align
            -Woverloaded-virtual -Wconversion -Wsign-conversion
            -Wpessimizing-move -Wredundant-move -Werror=unused-value
            -Werror=unused-variable -Werror=unused-parameter
    )
endif()