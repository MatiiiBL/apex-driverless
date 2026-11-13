# Tests may link apex::sim to generate realistic data, even for the car's modules. The support
# library holds the OSQP oracle, the test plant and the reference builders; the suite needs
# OSQP for its solver checks, so it is built only when OSQP is.
if(TARGET osqpstatic)
    add_subdirectory(support)
    apex_tests(control
        SOURCES
            components_test.cpp
            config_test.cpp
            controller_test.cpp
            mpc_test.cpp
            qp_solver_test.cpp
        DEPS apex_control_support)
endif()
