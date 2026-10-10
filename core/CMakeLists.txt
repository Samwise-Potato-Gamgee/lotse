# Defines lotse_core: the plain C++20 library with no ROS dependency.

add_library(lotse_core
    src/cbs.cpp
    src/clock.cpp
    src/continuous_benchmark.cpp
    src/continuous_runner.cpp
    src/distance.cpp
    src/executor.cpp
    src/fleet_manager.cpp
    src/geometry.cpp
    src/grid.cpp
    src/grid_simulator.cpp
    src/judge.cpp
    src/map.cpp
    src/mapf_benchmark.cpp
    src/order.cpp
    src/pass_order.cpp
    src/prioritized_planner.cpp
    src/reservations.cpp
    src/safety_benchmark.cpp
    src/space_time_search.cpp
    src/task_assigner.cpp)

add_library(lotse::core ALIAS lotse_core)

target_include_directories(lotse_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(lotse_core
    PRIVATE
        lotse_project_warnings)
