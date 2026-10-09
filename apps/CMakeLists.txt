# Command line tools: the safety benchmark and the delayed-run demo.

find_package(Threads REQUIRED)

add_executable(safety_benchmark safety_benchmark_main.cpp)
target_link_libraries(safety_benchmark
    PRIVATE
        lotse::core
        lotse_project_warnings
        Threads::Threads)

add_executable(replay_demo replay_demo_main.cpp)
target_link_libraries(replay_demo
    PRIVATE
        lotse::core
        lotse_project_warnings)
