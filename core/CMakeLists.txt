# lotse_core: the plain C++20 library with no ROS dependency. The ROS 2
# packages and the grid simulator are thin wrappers around this.
add_library(lotse_core
    src/grid.cpp)

add_library(lotse::core ALIAS lotse_core)

target_include_directories(lotse_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(lotse_core
    PRIVATE
        lotse_project_warnings)
