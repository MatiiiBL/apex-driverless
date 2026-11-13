# Tests may link apex::sim to generate realistic data, even for the car's modules. The QP tests
# check the module's solver against OSQP, so they are built only when OSQP is.
if(TARGET osqpstatic)
    add_subdirectory(support)
    apex_tests(control
        SOURCES
            qp_solver_test.cpp
        DEPS apex::control apex_control_support)
endif()
