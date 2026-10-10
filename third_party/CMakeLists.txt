# Fetches the three pinned third-party libraries used by the service and exposes
# them as one SYSTEM include target. Downloads land in .build/_deps (gitignored)
# and are verified by SHA256, so a rebuild is reproducible and offline once the
# cache is warm.

set(LOTSE_DEPS_DIR "${CMAKE_SOURCE_DIR}/.build/_deps" CACHE PATH
    "Cache for the downloaded third-party sources")

find_package(Threads REQUIRED)

set(LOTSE_JSON_URL "https://raw.githubusercontent.com/nlohmann/json/v3.12.0/single_include/nlohmann/json.hpp")
set(LOTSE_JSON_SHA "aaf127c04cb31c406e5b04a63f1ae89369fccde6d8fa7cdda1ed4f32dfc5de63")

set(LOTSE_HTTPLIB_URL "https://raw.githubusercontent.com/yhirose/cpp-httplib/v0.60.1/httplib.h")
set(LOTSE_HTTPLIB_SHA "e932d7771bdb93150868987e30112d9a31e24bef592d6d7a07e973d5845cf8d6")

set(LOTSE_SQLITE_URL "https://www.sqlite.org/2026/sqlite-amalgamation-3540000.zip")
set(LOTSE_SQLITE_SHA "68e913b0fe8ec6b4e5f391c594a28872884e80d6cd042a1e50c957be46b5aa4d")

file(MAKE_DIRECTORY "${LOTSE_DEPS_DIR}/include/nlohmann")

# Downloads a file only when it is not cached, then checks its SHA256.
# @param url the pinned source URL
# @param hash the expected SHA256
# @param output the cached file path
function(lotse_fetch_file url hash output)
    if(EXISTS "${output}")
        file(SHA256 "${output}" current_hash)
        if(current_hash STREQUAL hash)
            return()
        endif()
        file(REMOVE "${output}")
    endif()
    message(STATUS "Lotse: downloading ${url}")
    file(DOWNLOAD "${url}" "${output}"
        EXPECTED_HASH "SHA256=${hash}"
        TLS_VERIFY ON
        SHOW_PROGRESS)
endfunction()

lotse_fetch_file("${LOTSE_JSON_URL}" "${LOTSE_JSON_SHA}"
    "${LOTSE_DEPS_DIR}/include/nlohmann/json.hpp")
lotse_fetch_file("${LOTSE_HTTPLIB_URL}" "${LOTSE_HTTPLIB_SHA}"
    "${LOTSE_DEPS_DIR}/include/httplib.h")

set(LOTSE_SQLITE_ZIP "${LOTSE_DEPS_DIR}/sqlite-amalgamation-3540000.zip")
set(LOTSE_SQLITE_SRC "${LOTSE_DEPS_DIR}/sqlite-amalgamation-3540000")
if(NOT EXISTS "${LOTSE_SQLITE_SRC}/sqlite3.c")
    lotse_fetch_file("${LOTSE_SQLITE_URL}" "${LOTSE_SQLITE_SHA}" "${LOTSE_SQLITE_ZIP}")
    file(ARCHIVE_EXTRACT INPUT "${LOTSE_SQLITE_ZIP}" DESTINATION "${LOTSE_DEPS_DIR}")
endif()

add_library(lotse_sqlite STATIC "${LOTSE_SQLITE_SRC}/sqlite3.c")
target_include_directories(lotse_sqlite SYSTEM PUBLIC "${LOTSE_SQLITE_SRC}")
target_compile_definitions(lotse_sqlite PRIVATE
    SQLITE_THREADSAFE=1
    SQLITE_OMIT_LOAD_EXTENSION=1
    SQLITE_DQS=0)

add_library(lotse_third_party INTERFACE)
target_include_directories(lotse_third_party SYSTEM INTERFACE
    "${LOTSE_DEPS_DIR}/include")
target_link_libraries(lotse_third_party INTERFACE lotse_sqlite Threads::Threads)
