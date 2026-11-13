# Simulator benchmarks and validation reports (Release): the numbers docs/sim.md quotes.
foreach(name sim_bench vehicle_validation)
    add_executable(apex_${name} ${name}.cpp)
    target_link_libraries(apex_${name} PRIVATE apex::sim apex_warnings)
endforeach()
