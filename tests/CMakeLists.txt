# GoogleTest unit tests for lotse_core.

find_package(GTest REQUIRED)

add_executable(lotse_core_tests
    benchmark/mapf_scen_test.cpp
    benchmark/randomized_cbs_test.cpp
    benchmark/randomized_continuous_test.cpp
    benchmark/randomized_safety_test.cpp
    core/cbs_test.cpp
    core/continuous_runner_test.cpp
    core/distance_test.cpp
    core/executor_test.cpp
    core/grid_test.cpp
    core/geometry_test.cpp
    core/map_layout_test.cpp
    core/map_test.cpp
    core/order_test.cpp
    core/planning_test.cpp
    core/task_assigner_test.cpp
    sim/grid_simulator_test.cpp)

target_compile_definitions(lotse_core_tests PRIVATE LOTSE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

target_link_libraries(lotse_core_tests
    PRIVATE
        lotse::core
        GTest::gtest
        GTest::gtest_main
        lotse_project_warnings)

include(GoogleTest)
gtest_discover_tests(lotse_core_tests)

add_executable(lotse_service_tests
    service/http_api_test.cpp
    service/service_test.cpp)

target_compile_definitions(lotse_service_tests PRIVATE LOTSE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

target_link_libraries(lotse_service_tests
    PRIVATE
        lotse::service
        GTest::gtest
        GTest::gtest_main
        lotse_project_warnings)

gtest_discover_tests(lotse_service_tests)
