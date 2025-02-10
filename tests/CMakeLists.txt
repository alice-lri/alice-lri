find_package(GTest REQUIRED)
add_executable(run_tests tests.cpp)
target_link_libraries(run_tests PRIVATE accurate_ri GTest::GTest GTest::Main)
add_test(NAME accurate_ri_tests COMMAND run_tests)
