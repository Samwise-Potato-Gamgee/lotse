# Top-level build: the C++20 core library, its unit tests and the tools.

cmake_minimum_required(VERSION 3.22)

project(lotse
    VERSION 0.1.0
    DESCRIPTION "Multi-robot warehouse fleet manager"
    LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

option(LOTSE_BUILD_TESTS "Build the unit tests" ON)
option(LOTSE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
set(LOTSE_SANITIZERS "" CACHE STRING
    "Comma separated sanitizers for build+link, e.g. address,undefined or thread")

# Warnings interface applied to every first-party target.
add_library(lotse_project_warnings INTERFACE)
target_compile_options(lotse_project_warnings INTERFACE
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wnon-virtual-dtor)
if(LOTSE_WARNINGS_AS_ERRORS)
    target_compile_options(lotse_project_warnings INTERFACE -Werror)
endif()

if(LOTSE_SANITIZERS)
    message(STATUS "Lotse: building with sanitizers = ${LOTSE_SANITIZERS}")
    add_compile_options(-fsanitize=${LOTSE_SANITIZERS} -fno-omit-frame-pointer)
    add_link_options(-fsanitize=${LOTSE_SANITIZERS})
endif()

add_subdirectory(third_party)
add_subdirectory(core)
add_subdirectory(service)
add_subdirectory(apps)

if(LOTSE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
