find_package(GTest REQUIRED)

add_executable(lotse_core_tests
    core/grid_test.cpp)

target_link_libraries(lotse_core_tests
    PRIVATE
        lotse::core
        GTest::gtest
        GTest::gtest_main
        lotse_project_warnings)

include(GoogleTest)
gtest_discover_tests(lotse_core_tests)
