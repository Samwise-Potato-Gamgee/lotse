cmake_minimum_required(VERSION 3.22)
project(lotse_link_check)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(link_check_node src/link_check_node.cpp)
target_compile_options(link_check_node PRIVATE
    -Wall -Wextra -Wpedantic -Wshadow -Werror)
ament_target_dependencies(link_check_node
    rclcpp
    geometry_msgs
    nav_msgs)

install(TARGETS link_check_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
