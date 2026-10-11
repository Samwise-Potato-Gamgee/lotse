# The service library (config, SQLite store, runtime, HTTP/WebSocket API) and
# the executable that serves the dashboard. Kept apart from lotse_core so the
# core stays free of third-party dependencies.

add_library(lotse_service
    src/config.cpp
    src/fleet_json.cpp
    src/http_api.cpp
    src/runtime.cpp
    src/sqlite_order_store.cpp)

add_library(lotse::service ALIAS lotse_service)

target_include_directories(lotse_service
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(lotse_service
    PUBLIC
        lotse::core
        lotse_third_party
    PRIVATE
        lotse_project_warnings)

add_executable(lotse_service_app src/main.cpp)
target_link_libraries(lotse_service_app
    PRIVATE
        lotse::service
        lotse_project_warnings)
