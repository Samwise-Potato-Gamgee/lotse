# Command line tools: the benchmarks, the terminal demos and the planner
# comparison.

find_package(Threads REQUIRED)

add_executable(safety_benchmark safety_benchmark_main.cpp)
target_link_libraries(safety_benchmark
    PRIVATE
        lotse::core
        lotse_project_warnings
        Threads::Threads)

add_executable(continuous_benchmark continuous_benchmark_main.cpp)
target_link_libraries(continuous_benchmark
    PRIVATE
        lotse::core
        lotse_project_warnings
        Threads::Threads)

add_executable(liveness_benchmark liveness_benchmark_main.cpp)
target_link_libraries(liveness_benchmark
    PRIVATE
        lotse::core
        lotse_project_warnings
        Threads::Threads)

add_executable(continuous_demo continuous_demo_main.cpp)
target_link_libraries(continuous_demo
    PRIVATE
        lotse::core
        lotse_project_warnings)

add_executable(planner_comparison planner_comparison_main.cpp)
target_link_libraries(planner_comparison
    PRIVATE
        lotse::core
        lotse_project_warnings
        Threads::Threads)

add_executable(replay_demo replay_demo_main.cpp)
target_link_libraries(replay_demo
    PRIVATE
        lotse::core
        lotse_project_warnings)
