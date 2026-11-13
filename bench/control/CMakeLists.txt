# Control benchmarks (Release): the MPC's QP solver against OSQP on the controller's own
# problems, and the closed-loop evaluation of the three controllers on the simulator's vehicle.
# Both write their results to results/control/ (docs/control.md).
if(NOT TARGET osqpstatic)
    return()
endif()
if(NOT TARGET apex_control_support)
    add_subdirectory(${PROJECT_SOURCE_DIR}/tests/control/support
        ${CMAKE_CURRENT_BINARY_DIR}/support)
endif()
foreach(bench qp_bench closed_loop_eval)
    add_executable(apex_control_${bench} ${bench}.cpp)
    target_link_libraries(apex_control_${bench} PRIVATE apex_control_support apex_warnings)
endforeach()
