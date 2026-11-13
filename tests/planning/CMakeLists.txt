# Tests may link apex::sim to generate realistic data, even for the car's modules.

# Imperfect cone maps and lap-1 drives made from a track's true cones; the evaluations in
# bench/planning use them too.
add_library(apex_planning_test_support STATIC support/cone_maps.cpp)
target_include_directories(apex_planning_test_support PUBLIC support)
target_link_libraries(apex_planning_test_support PUBLIC apex::core PRIVATE apex_warnings)

set(planning_test_sources
    config_json_test.cpp
    delaunay_test.cpp
    predicates_test.cpp
    qp_solver_test.cpp
    raceline_test.cpp
    smoothing_test.cpp
    speed_profile_test.cpp
    track_model_test.cpp)
set(planning_test_deps apex::sim apex_planning_test_support)

# OSQP is the reference the module's QP solver is checked against (design.md, section 7).
if(TARGET osqpstatic)
    add_library(apex_planning_osqp_reference STATIC support/osqp_reference.cpp)
    target_include_directories(apex_planning_osqp_reference PUBLIC support)
    target_link_libraries(apex_planning_osqp_reference
        PUBLIC apex::planning osqpstatic
        PRIVATE apex_warnings)
    list(APPEND planning_test_sources qp_osqp_test.cpp)
    list(APPEND planning_test_deps apex_planning_osqp_reference)
endif()

# apex::planning last: listed before the OSQP reference, which depends on it, the linker would
# see the archive twice.
apex_tests(planning
    SOURCES ${planning_test_sources}
    DEPS ${planning_test_deps} apex::planning)
