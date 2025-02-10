add_library(accurate_ri SHARED src/accurate_ri.cpp)
target_include_directories(accurate_ri PUBLIC include)

install(TARGETS accurate_ri DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)