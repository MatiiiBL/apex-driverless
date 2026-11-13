# Simulator benchmarks and validation reports (Release): the numbers docs/sim.md quotes, and the
# track export behind its gallery.
foreach(name sim_bench vehicle_validation export_tracks)
    add_executable(apex_${name} ${name}.cpp)
    target_link_libraries(apex_${name} PRIVATE apex::sim apex_warnings)
endforeach()
