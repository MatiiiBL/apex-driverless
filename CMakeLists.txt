cmake_minimum_required(VERSION 3.25)

project(apex
    VERSION 0.1.0
    DESCRIPTION "An autonomous Formula Student Driverless stack in C++20"
    LANGUAGES C CXX)

# -std=c++20 rather than the GNU dialect.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(APEX_BUILD_TESTS "Build the test suites" ${PROJECT_IS_TOP_LEVEL})
option(APEX_BUILD_BENCHMARKS "Build the benchmarks (meaningful in Release only)" OFF)
option(APEX_SANITIZE "Instrument everything with AddressSanitizer + UBSan" OFF)
option(APEX_WARNINGS_AS_ERRORS "Treat compiler warnings in apex code as errors" ON)
# OSQP is the reference QP solver that the planning and control solvers are checked and
# benchmarked against. Library code never links it; only tests and benchmarks do.
option(APEX_WITH_OSQP "Build OSQP for the reference-solver tests and benchmarks" ON)

# Sanitizers are directory-wide on purpose: every target, including GoogleTest, must be
# instrumented, and the flags must reach the linker too. This has to run before any target is
# created. -fno-sanitize-recover makes UBSan abort instead of printing and passing.
if(APEX_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=all
        -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

include(cmake/Dependencies.cmake)

# Warnings are per-target: they apply to apex's own code only, never to dependencies.
add_library(apex_warnings INTERFACE)
target_compile_options(apex_warnings INTERFACE
    -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wnon-virtual-dtor -Wold-style-cast
    $<$<BOOL:${APEX_WARNINGS_AS_ERRORS}>:-Werror>)

include(cmake/ApexModule.cmake)

# The car's software (perception, estimation, planning, control, stack) never depends on the
# simulator: only the runner, the apps and the tests see both sides.
add_subdirectory(src/core)
add_subdirectory(src/sim)
add_subdirectory(src/perception)
add_subdirectory(src/estimation)
add_subdirectory(src/planning)
add_subdirectory(src/control)
add_subdirectory(src/stack)
add_subdirectory(src/runner)
add_subdirectory(apps)

if(APEX_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(APEX_BUILD_BENCHMARKS)
    add_subdirectory(bench)
endif()
