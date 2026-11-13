# Tests may link apex::sim to generate realistic data, even for the car's modules: the closed-
# loop tests drive the simulator's vehicle model. The support library holds it with the OSQP
# oracle and the reference builders; the suite needs OSQP for its solver checks, so it is built
# only when OSQP is. apex::planning builds references with the stack's own speed profiles.
if(TARGET osqpstatic)
    add_subdirectory(support)
    apex_tests(control
        SOURCES
            components_test.cpp
            config_test.cpp
            controller_test.cpp
            mpc_test.cpp
            qp_solver_test.cpp
        DEPS apex_control_support apex::planning)
endif()
