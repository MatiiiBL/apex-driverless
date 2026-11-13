# Planning evaluations and benchmarks (Release): the numbers docs/planning.md quotes. They use
# the planning tests' imperfect maps, evaluation and OSQP reference (tests/planning/support).
if(NOT TARGET apex_planning_test_support)
    message(STATUS "bench/planning: needs APEX_BUILD_TESTS for tests/planning/support; skipped")
    return()
endif()

foreach(name exploration_eval raceline_eval)
    add_executable(apex_planning_${name} ${name}.cpp)
    target_link_libraries(apex_planning_${name}
        PRIVATE apex_planning_test_support apex::sim apex_warnings)
endforeach()

if(TARGET apex_planning_osqp_reference)
    add_executable(apex_planning_qp_bench qp_bench.cpp)
    target_link_libraries(apex_planning_qp_bench
        PRIVATE apex_planning_osqp_reference apex::sim apex_warnings)
endif()
