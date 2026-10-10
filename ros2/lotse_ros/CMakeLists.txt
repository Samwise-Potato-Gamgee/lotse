# ROS 2 package: the robot side adapter, the driver node, the stand-in
# simulator and the independent contact judge. The core library is plain CMake
# and is compiled in here, so the core stays free of any ROS dependency.

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

if(NOT TARGET lotse_project_warnings)
    add_library(lotse_project_warnings INTERFACE)
    target_compile_options(lotse_project_warnings INTERFACE
        -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Werror)
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core
    ${CMAKE_CURRENT_BINARY_DIR}/lotse_core)

set(LOTSE_ROS_MSG_DEPS rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs)

add_library(lotse_ros_side src/ros_robot_side.cpp)
target_include_directories(lotse_ros_side PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(lotse_ros_side ${LOTSE_ROS_MSG_DEPS})
target_link_libraries(lotse_ros_side lotse::core lotse_project_warnings)

foreach(app driver standin_sim fleet_node judge_node)
    add_executable(${app} src/${app}_main.cpp)
    ament_target_dependencies(${app} ${LOTSE_ROS_MSG_DEPS})
    target_link_libraries(${app} lotse_ros_side lotse_project_warnings)
endforeach()

install(TARGETS lotse_ros_side driver standin_sim fleet_node judge_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
