include(FetchContent)

FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
    URL_HASH SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SYSTEM)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

add_subdirectory(core)
add_subdirectory(sim)
add_subdirectory(perception)
add_subdirectory(estimation)
add_subdirectory(planning)
add_subdirectory(control)
add_subdirectory(stack)
add_subdirectory(runner)
