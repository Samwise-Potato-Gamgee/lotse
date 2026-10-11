# ROS 2 package: the robot side adapter, the driver node, the stand-in
# simulator, the independent contact judge, the dashboard fleet node and the
# Isaac scene launch. The core library is plain CMake and is compiled in here,
# so the core stays free of any ROS dependency.

cmake_minimum_required(VERSION 3.22)
project(lotse_ros)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(Threads REQUIRED)

if(NOT TARGET lotse_project_warnings)
    add_library(lotse_project_warnings INTERFACE)
    target_compile_options(lotse_project_warnings INTERFACE
        -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Werror)
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core
    ${CMAKE_CURRENT_BINARY_DIR}/lotse_core)

set(LOTSE_ROS_MSG_DEPS rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs rosgraph_msgs)

set(LOTSE_DEPS_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/../../.build/_deps/include")
set(LOTSE_SERVICE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../service")

file(GLOB LOTSE_SQLITE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../../.build/_deps/sqlite-amalgamation-*/sqlite3.c")
list(GET LOTSE_SQLITE_SOURCES 0 LOTSE_SQLITE_C)
get_filename_component(LOTSE_SQLITE_DIR "${LOTSE_SQLITE_C}" DIRECTORY)

add_library(lotse_sqlite_shared STATIC "${LOTSE_SQLITE_C}")
target_include_directories(lotse_sqlite_shared SYSTEM PUBLIC "${LOTSE_SQLITE_DIR}")
target_compile_definitions(lotse_sqlite_shared PRIVATE
    SQLITE_THREADSAFE=1 SQLITE_OMIT_LOAD_EXTENSION=1 SQLITE_DQS=0)

add_library(lotse_service_shared STATIC
    ${LOTSE_SERVICE_DIR}/src/fleet_json.cpp
    ${LOTSE_SERVICE_DIR}/src/http_api.cpp
    ${LOTSE_SERVICE_DIR}/src/sqlite_order_store.cpp)
target_include_directories(lotse_service_shared PUBLIC "${LOTSE_SERVICE_DIR}/include")
target_include_directories(lotse_service_shared SYSTEM PUBLIC "${LOTSE_DEPS_INCLUDE}")
target_link_libraries(lotse_service_shared
    PUBLIC lotse::core lotse_sqlite_shared Threads::Threads
    PRIVATE lotse_project_warnings)

add_library(lotse_ros_side src/ros_robot_side.cpp)
target_include_directories(lotse_ros_side PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(lotse_ros_side SYSTEM PUBLIC "${LOTSE_DEPS_INCLUDE}")
ament_target_dependencies(lotse_ros_side ${LOTSE_ROS_MSG_DEPS})
target_link_libraries(lotse_ros_side lotse::core lotse_project_warnings)

foreach(app driver standin_sim fleet_node fleet_api judge_node)
    add_executable(${app} src/${app}_main.cpp)
    ament_target_dependencies(${app} ${LOTSE_ROS_MSG_DEPS})
    target_link_libraries(${app} lotse_ros_side lotse_project_warnings)
endforeach()

target_link_libraries(fleet_api lotse_service_shared)

install(TARGETS lotse_ros_side driver standin_sim fleet_node fleet_api judge_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
