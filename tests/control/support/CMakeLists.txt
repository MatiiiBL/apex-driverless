# Support code shared by the control tests and benchmarks (bench/control/ adds this directory
# when the tests are not built): the OSQP reference oracle and random MPC-structured QPs. The
# control library itself never links any of it.
add_library(apex_control_support STATIC osqp_reference.cpp)
target_include_directories(apex_control_support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(apex_control_support
    PUBLIC apex::control
    PRIVATE osqpstatic apex_warnings)
